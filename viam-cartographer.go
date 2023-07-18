// Package viamcartographer implements simultaneous localization and mapping.
// This is an Experimental package.
package viamcartographer

import (
	"bufio"
	"bytes"
	"context"
	"io"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.uber.org/zap/zapcore"
	pb "go.viam.com/api/service/slam/v1"
	viamgrpc "go.viam.com/rdk/grpc"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/services/slam/grpchelper"
	"go.viam.com/rdk/spatialmath"
	goutils "go.viam.com/utils"
	"go.viam.com/utils/pexec"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	"github.com/viamrobotics/viam-cartographer/sensorprocess"
	"github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
	dim2d "github.com/viamrobotics/viam-cartographer/sensors/lidar/dim-2d"
	vcUtils "github.com/viamrobotics/viam-cartographer/utils"
)

// Model is the model name of cartographer.
var (
	Model    = resource.NewModel("viam", "slam", "cartographer")
	cartoLib cartofacade.CartoLib
	// ErrClosed denotes that the slam service method was called on a closed slam resource.
	ErrClosed = errors.Errorf("resource (%s) is closed", Model.String())
)

const (
	// DefaultExecutableName is what this program expects to call to start the cartographer grpc server.
	DefaultExecutableName                = "carto_grpc_server"
	defaultDataRateMsec                  = 200
	defaultMapRateSec                    = 60
	defaultDialMaxTimeoutSec             = 30
	defaultSensorValidationMaxTimeoutSec = 30
	defaultSensorValidationIntervalSec   = 1
	parsePortMaxTimeoutSec               = 60
	localhost0                           = "localhost:0"
	defaultCartoFacadeTimeout            = 5 * time.Second
	chunkSizeBytes                       = 1 * 1024 * 1024
)

var defaultCartoAlgoCfg = cartofacade.CartoAlgoConfig{
	OptimizeOnStart:      false,
	OptimizeEveryNNodes:  3,
	NumRangeData:         30,
	MissingDataRayLength: 25.0,
	MaxRange:             25.0,
	MinRange:             0.2,
	MaxSubmapsToKeep:     3,
	FreshSubmapsCount:    3,
	MinCoveredArea:       1.0,
	MinAddedSubmapsCount: 1,
	OccupiedSpaceWeight:  20.0,
	TranslationWeight:    10.0,
	RotationWeight:       1.0,
}

// SubAlgo defines the cartographer specific sub-algorithms that we support.
type SubAlgo string

// Dim2d runs cartographer with a 2D LIDAR only.
const Dim2d SubAlgo = "2d"

func init() {
	resource.RegisterService(slam.API, Model, resource.Registration[slam.Service, *vcConfig.Config]{
		Constructor: func(
			ctx context.Context,
			deps resource.Dependencies,
			c resource.Config,
			logger golog.Logger,
		) (slam.Service, error) {
			return New(
				ctx,
				deps,
				c,
				logger,
				false,
				DefaultExecutableName,
				defaultSensorValidationMaxTimeoutSec,
				defaultSensorValidationIntervalSec,
				defaultDialMaxTimeoutSec,
				defaultCartoFacadeTimeout,
				nil,
			)
		},
	})
}

// InitCartoLib is run to initialize the cartographer library
// must be called before module.AddModelFromRegistry is
// called.
func InitCartoLib(logger golog.Logger) error {
	minloglevel := 1 // warn
	vlog := 0        //  disabled
	if logger.Level() == zapcore.DebugLevel {
		minloglevel = 0 // info
		vlog = 1        // verbose enabled
	}
	lib, err := cartofacade.NewLib(minloglevel, vlog)
	if err != nil {
		return err
	}
	cartoLib = lib
	return nil
}

// TerminateCartoLib is run to terminate the cartographer library.
func TerminateCartoLib() error {
	return cartoLib.Terminate()
}

func initSensorProcess(cancelCtx context.Context, cartoSvc *CartographerService) {
	spConfig := sensorprocess.Config{
		CartoFacade:      cartoSvc.cartofacade,
		Lidar:            cartoSvc.timedLidar,
		LidarName:        cartoSvc.primarySensorName,
		DataRateMs:       cartoSvc.dataRateMs,
		Timeout:          cartoSvc.cartoFacadeTimeout,
		Logger:           cartoSvc.logger,
		TelemetryEnabled: cartoSvc.logger.Level() == zapcore.DebugLevel,
	}

	cartoSvc.sensorProcessWorkers.Add(1)
	go func() {
		defer cartoSvc.sensorProcessWorkers.Done()
		if jobDone := sensorprocess.Start(cancelCtx, spConfig); jobDone {
			cartoSvc.jobDone = true
			cartoSvc.cancelSensorProcessFunc()
		}
	}()
}

// New returns a new slam service for the given robot.
func New(
	ctx context.Context,
	deps resource.Dependencies,
	c resource.Config,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
	executableName string,
	sensorValidationMaxTimeoutSec int,
	sensorValidationIntervalSec int,
	dialMaxTimeoutSec int,
	cartoFacadeTimeout time.Duration,
	testTimedSensorOverride sensors.TimedSensor,
) (slam.Service, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::slamService::New")
	defer span.End()

	svcConfig, err := resource.NativeConfig[*vcConfig.Config](c)
	if err != nil {
		return nil, err
	}

	subAlgo := SubAlgo(svcConfig.ConfigParams["mode"])
	if subAlgo != Dim2d {
		return nil, errors.Errorf("%v does not have a 'mode: %v'",
			c.Model.Name, svcConfig.ConfigParams["mode"])
	}

	port, dataRateMsec, mapRateSec, useLiveData, deleteProcessedData, modularizationV2Enabled, err := vcConfig.GetOptionalParameters(
		svcConfig,
		localhost0,
		defaultDataRateMsec,
		defaultMapRateSec,
		logger,
	)
	if err != nil {
		return nil, err
	}

	if !modularizationV2Enabled {
		if err := vcConfig.SetupDirectories(svcConfig.DataDirectory, logger); err != nil {
			return nil, err
		}
	}

	// Get the lidar for the Dim2D cartographer sub algorithm
	lidar, err := dim2d.NewLidar(ctx, deps, svcConfig.Sensors, logger)
	if err != nil {
		return nil, err
	}

	// Need to pass in a long-lived context because ctx is short-lived
	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	// Need to be able to shut down the sensor process before the cartoFacade
	cancelSensorProcessCtx, cancelSensorProcessFunc := context.WithCancel(context.Background())
	cancelCartoFacadeCtx, cancelCartoFacadeFunc := context.WithCancel(context.Background())

	// use the override in testing if non nil
	// otherwise use the lidar from deps as the
	// timed sensor
	timedSensor := testTimedSensorOverride
	if timedSensor == nil {
		timedSensor = lidar
	}

	// Cartographer SLAM Service Object
	cartoSvc := &CartographerService{
		Named:                         c.ResourceName().AsNamed(),
		primarySensorName:             lidar.Name,
		lidar:                         lidar,
		timedLidar:                    timedSensor,
		executableName:                executableName,
		subAlgo:                       subAlgo,
		slamProcess:                   pexec.NewProcessManager(logger),
		configParams:                  svcConfig.ConfigParams,
		dataDirectory:                 svcConfig.DataDirectory,
		sensors:                       svcConfig.Sensors,
		useLiveData:                   useLiveData,
		deleteProcessedData:           deleteProcessedData,
		port:                          port,
		dataRateMs:                    dataRateMsec,
		mapRateSec:                    mapRateSec,
		cancelFunc:                    cancelFunc,
		cancelSensorProcessFunc:       cancelSensorProcessFunc,
		cancelCartoFacadeFunc:         cancelCartoFacadeFunc,
		logger:                        logger,
		bufferSLAMProcessLogs:         bufferSLAMProcessLogs,
		modularizationV2Enabled:       modularizationV2Enabled,
		sensorValidationMaxTimeoutSec: sensorValidationMaxTimeoutSec,
		sensorValidationIntervalSec:   sensorValidationMaxTimeoutSec,
		dialMaxTimeoutSec:             dialMaxTimeoutSec,
		cartoFacadeTimeout:            cartoFacadeTimeout,
		localizationMode:              mapRateSec == 0,
		mapTimestamp:                  time.Now().UTC(),
		jobDone:                       false,
	}
	defer func() {
		if err != nil {
			logger.Errorw("New() hit error, closing...", "error", err)
			if err := cartoSvc.Close(ctx); err != nil {
				logger.Errorw("error closing out after error", "error", err)
			}
		}
	}()

	if modularizationV2Enabled {
		if err = sensors.ValidateGetData(
			cancelSensorProcessCtx,
			timedSensor,
			time.Duration(sensorValidationMaxTimeoutSec)*time.Second,
			time.Duration(cartoSvc.sensorValidationIntervalSec)*time.Second,
			cartoSvc.logger); err != nil {
			err = errors.Wrap(err, "failed to get data from lidar")
			return nil, err
		}

		err = initCartoFacade(cancelCartoFacadeCtx, cartoSvc)
		if err != nil {
			return nil, err
		}

		initSensorProcess(cancelSensorProcessCtx, cartoSvc)
		return cartoSvc, nil
	}

	err = initCartoGrpcServer(ctx, cancelCtx, cartoSvc)
	if err != nil {
		return nil, err
	}
	return cartoSvc, nil
}

func parseCartoAlgoConfig(configParams map[string]string, logger golog.Logger) (cartofacade.CartoAlgoConfig, error) {
	cartoAlgoCfg := defaultCartoAlgoCfg
	for k, val := range configParams {
		switch k {
		case "optimize_on_start":
			if val == "true" {
				cartoAlgoCfg.OptimizeOnStart = true
			}
		case "optimize_every_n_nodes":
			iVal, err := strconv.Atoi(val)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.OptimizeEveryNNodes = iVal
		case "num_range_data":
			iVal, err := strconv.Atoi(val)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.NumRangeData = iVal
		case "missing_data_ray_length":
			fVal, err := strconv.ParseFloat(val, 32)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.MissingDataRayLength = float32(fVal)
		case "max_range":
			fVal, err := strconv.ParseFloat(val, 32)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.MaxRange = float32(fVal)
		case "min_range":
			fVal, err := strconv.ParseFloat(val, 32)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.MinRange = float32(fVal)
		case "max_submaps_to_keep":
			iVal, err := strconv.Atoi(val)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.MaxSubmapsToKeep = iVal
		case "fresh_submaps_count":
			iVal, err := strconv.Atoi(val)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.FreshSubmapsCount = iVal
		case "min_covered_area":
			fVal, err := strconv.ParseFloat(val, 64)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.MinCoveredArea = fVal
		case "min_added_submaps_count":
			iVal, err := strconv.Atoi(val)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.MinAddedSubmapsCount = iVal
		case "occupied_space_weight":
			fVal, err := strconv.ParseFloat(val, 64)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.OccupiedSpaceWeight = fVal
		case "translation_weight":
			fVal, err := strconv.ParseFloat(val, 64)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.TranslationWeight = fVal
		case "rotation_weight":
			fVal, err := strconv.ParseFloat(val, 64)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.RotationWeight = fVal
			// ignore mode as it is a special case
		case "mode":
		default:
			logger.Warnf("unused config param: %s: %s", k, val)
		}
	}
	return cartoAlgoCfg, nil
}

// initCartoFacade
// 1. creates a new initCartoFacade
// 2. initializes it and starts it
// 3. terminates it if start fails.
func initCartoFacade(ctx context.Context, cartoSvc *CartographerService) error {
	cartoAlgoConfig, err := parseCartoAlgoConfig(cartoSvc.configParams, cartoSvc.logger)
	if err != nil {
		return err
	}

	cartoCfg := cartofacade.CartoConfig{
		Sensors:            cartoSvc.sensors,
		MapRateSecond:      cartoSvc.mapRateSec,
		DataDir:            cartoSvc.dataDirectory,
		ComponentReference: cartoSvc.primarySensorName,
		LidarConfig:        cartofacade.TwoD,
	}

	cf := cartofacade.New(&cartoLib, cartoCfg, cartoAlgoConfig)
	slamMode, err := cf.Initialize(ctx, cartoSvc.cartoFacadeTimeout, &cartoSvc.cartoFacadeWorkers)
	if err != nil {
		cartoSvc.logger.Errorw("cartofacade initialize failed", "error", err)
		return err
	}
	err = cf.Start(ctx, cartoSvc.cartoFacadeTimeout)
	if err != nil {
		cartoSvc.logger.Errorw("cartofacade start failed", "error", err)
		termErr := cf.Terminate(ctx, cartoSvc.cartoFacadeTimeout)
		if termErr != nil {
			cartoSvc.logger.Errorw("cartofacade terminate failed", "error", termErr)
			return termErr
		}
		return err
	}

	cartoSvc.cartofacade = &cf
	cartoSvc.SlamMode = slamMode

	return nil
}

func terminateCartoFacade(ctx context.Context, cartoSvc *CartographerService) error {
	if cartoSvc.cartofacade == nil {
		cartoSvc.logger.Debug("terminateCartoFacade called when cartoSvc.cartofacade is nil")
		return nil
	}
	stopErr := cartoSvc.cartofacade.Stop(ctx, cartoSvc.cartoFacadeTimeout)
	if stopErr != nil {
		cartoSvc.logger.Errorw("cartofacade stop failed", "error", stopErr)
	}

	err := cartoSvc.cartofacade.Terminate(ctx, cartoSvc.cartoFacadeTimeout)
	if err != nil {
		cartoSvc.logger.Errorw("cartofacade terminate failed", "error", err)
		return err
	}
	return stopErr
}

func initCartoGrpcServer(ctx, cancelCtx context.Context, cartoSvc *CartographerService) error {
	if cartoSvc.primarySensorName != "" {
		if err := dim2d.ValidateGetAndSaveData(cancelCtx, cartoSvc.dataDirectory, cartoSvc.lidar,
			cartoSvc.sensorValidationMaxTimeoutSec, cartoSvc.sensorValidationIntervalSec, cartoSvc.logger); err != nil {
			return errors.Wrap(err, "getting and saving data failed")
		}
		cartoSvc.StartDataProcess(cancelCtx, cartoSvc.lidar, nil)
		cartoSvc.logger.Debugf("Reading data from sensor: %v", cartoSvc.primarySensorName)
	}

	if err := cartoSvc.StartSLAMProcess(ctx); err != nil {
		return errors.Wrap(err, "error with slam service slam process")
	}

	client, clientClose, err := vcConfig.SetupGRPCConnection(ctx, cartoSvc.port, cartoSvc.dialMaxTimeoutSec, cartoSvc.logger)
	if err != nil {
		return errors.Wrap(err, "error with initial grpc client to slam algorithm")
	}
	cartoSvc.clientAlgo = client
	cartoSvc.clientAlgoClose = clientClose

	return nil
}

// CartographerService is the structure of the slam service.
type CartographerService struct {
	resource.Named
	resource.AlwaysRebuild
	mu                sync.Mutex
	SlamMode          cartofacade.SlamMode
	closed            bool
	primarySensorName string
	lidar             lidar.Lidar
	timedLidar        sensors.TimedSensor
	executableName    string
	subAlgo           SubAlgo
	// deprecated
	slamProcess pexec.ProcessManager
	// deprecated
	clientAlgo pb.SLAMServiceClient
	// deprecated
	clientAlgoClose func() error

	configParams  map[string]string
	dataDirectory string
	sensors       []string
	// deprecated
	deleteProcessedData bool
	// deprecated
	useLiveData bool

	modularizationV2Enabled bool
	cartofacade             cartofacade.Interface
	cartoFacadeTimeout      time.Duration

	// deprecated
	port       string
	dataRateMs int
	mapRateSec int

	// deprecated
	cancelFunc              func()
	cancelSensorProcessFunc func()
	cancelCartoFacadeFunc   func()
	logger                  golog.Logger
	// deprecated
	activeBackgroundWorkers sync.WaitGroup
	sensorProcessWorkers    sync.WaitGroup
	cartoFacadeWorkers      sync.WaitGroup

	// deprecated
	bufferSLAMProcessLogs bool
	// deprecated
	slamProcessLogReader io.ReadCloser
	// deprecated
	slamProcessLogWriter io.WriteCloser
	// deprecated
	slamProcessBufferedLogReader bufio.Reader

	localizationMode              bool
	mapTimestamp                  time.Time
	sensorValidationMaxTimeoutSec int
	sensorValidationIntervalSec   int
	// deprecated
	dialMaxTimeoutSec int
	jobDone           bool
}

// GetPosition forwards the request for positional data to the slam library's gRPC service. Once a response is received,
// it is unpacked into a Pose and a component reference string.
func (cartoSvc *CartographerService) GetPosition(ctx context.Context) (spatialmath.Pose, string, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::GetPosition")
	defer span.End()
	if cartoSvc.closed {
		cartoSvc.logger.Warn("GetPosition called after closed")
		return nil, "", ErrClosed
	}

	if cartoSvc.modularizationV2Enabled {
		return cartoSvc.getPositionModularizationV2(ctx)
	}

	req := &pb.GetPositionRequest{Name: cartoSvc.Name().ShortName()}

	resp, err := cartoSvc.clientAlgo.GetPosition(ctx, req)
	if err != nil {
		return nil, "", errors.Wrap(err, "error getting SLAM position")
	}
	pose := spatialmath.NewPoseFromProtobuf(resp.GetPose())
	componentReference := resp.GetComponentReference()
	returnedExt := resp.Extra.AsMap()

	return vcUtils.CheckQuaternionFromClientAlgo(pose, componentReference, returnedExt)
}

func (cartoSvc *CartographerService) getPositionModularizationV2(ctx context.Context) (spatialmath.Pose, string, error) {
	pos, err := cartoSvc.cartofacade.GetPosition(ctx, cartoSvc.cartoFacadeTimeout)
	if err != nil {
		return nil, "", err
	}

	pose := spatialmath.NewPoseFromPoint(r3.Vector{X: pos.X, Y: pos.Y, Z: pos.Z})
	returnedExt := map[string]interface{}{
		"quat": map[string]interface{}{
			"real": pos.Real,
			"imag": pos.Imag,
			"jmag": pos.Jmag,
			"kmag": pos.Kmag,
		},
	}
	return vcUtils.CheckQuaternionFromClientAlgo(pose, cartoSvc.primarySensorName, returnedExt)
}

// GetPointCloudMap creates a request, recording the time, calls the slam algorithms GetPointCloudMap endpoint and returns a callback
// function which will return the next chunk of the current pointcloud map.
// If startup is in localization mode, the timestamp is NOT updated.
func (cartoSvc *CartographerService) GetPointCloudMap(ctx context.Context) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::GetPointCloudMap")
	defer span.End()

	if cartoSvc.closed {
		cartoSvc.logger.Warn("GetPointCloudMap called after closed")
		return nil, ErrClosed
	}

	if !cartoSvc.localizationMode {
		cartoSvc.mapTimestamp = time.Now().UTC()
	}

	if cartoSvc.modularizationV2Enabled {
		return cartoSvc.getPointCloudMapModularizationV2(ctx)
	}
	return grpchelper.GetPointCloudMapCallback(ctx, cartoSvc.Name().ShortName(), cartoSvc.clientAlgo)
}

func (cartoSvc *CartographerService) getPointCloudMapModularizationV2(ctx context.Context) (func() ([]byte, error), error) {
	chunk := make([]byte, chunkSizeBytes)
	pc, err := cartoSvc.cartofacade.GetPointCloudMap(ctx, cartoSvc.cartoFacadeTimeout)
	if err != nil {
		return nil, err
	}

	pointcloudReader := bytes.NewReader(pc)

	f := func() ([]byte, error) {
		bytesRead, err := pointcloudReader.Read(chunk)
		if err != nil {
			return nil, err
		}
		return chunk[:bytesRead], err
	}
	return f, nil
}

// GetInternalState creates a request, calls the slam algorithms GetInternalState endpoint and returns a callback
// function which will return the next chunk of the current internal state of the slam algo.
func (cartoSvc *CartographerService) GetInternalState(ctx context.Context) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::GetInternalState")
	defer span.End()

	if cartoSvc.closed {
		cartoSvc.logger.Warn("GetInternalState called after closed")
		return nil, ErrClosed
	}

	if cartoSvc.modularizationV2Enabled {
		return cartoSvc.getInternalStateModularizationV2Enabled(ctx)
	}

	return grpchelper.GetInternalStateCallback(ctx, cartoSvc.Name().ShortName(), cartoSvc.clientAlgo)
}

func (cartoSvc *CartographerService) getInternalStateModularizationV2Enabled(ctx context.Context) (func() ([]byte, error), error) {
	chunk := make([]byte, chunkSizeBytes)
	is, err := cartoSvc.cartofacade.GetInternalState(ctx, cartoSvc.cartoFacadeTimeout)
	if err != nil {
		return nil, err
	}

	internalStateReader := bytes.NewReader(is)

	f := func() ([]byte, error) {
		bytesRead, err := internalStateReader.Read(chunk)
		if err != nil {
			return nil, err
		}
		return chunk[:bytesRead], err
	}
	return f, nil
}

// GetLatestMapInfo returns the timestamp  associated with the latest call to GetPointCloudMap,
// unless you are localizing; in which case the timestamp returned is the timestamp of the session.
func (cartoSvc *CartographerService) GetLatestMapInfo(ctx context.Context) (time.Time, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::GetLatestMapInfo")
	defer span.End()

	if cartoSvc.closed {
		cartoSvc.logger.Warn("GetLatestMapInfo called after closed")
		return time.Time{}, ErrClosed
	}

	return cartoSvc.mapTimestamp, nil
}

// StartDataProcess starts a go routine that saves data from the lidar to the user-defined data directory.
func (cartoSvc *CartographerService) StartDataProcess(
	ctx context.Context,
	lidar lidar.Lidar,
	c chan int,
) {
	cartoSvc.activeBackgroundWorkers.Add(1)
	if err := ctx.Err(); err != nil {
		if !errors.Is(err, context.Canceled) {
			cartoSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
		}
		cartoSvc.activeBackgroundWorkers.Done()
		return
	}

	goutils.PanicCapturingGo(func() {
		if !cartoSvc.useLiveData {
			// If we're not using live data, we read from the sensor as fast as
			// possible, since the sensor is just playing back pre-captured data.
			cartoSvc.readData(ctx, lidar, c)
		} else {
			cartoSvc.readDataOnInterval(ctx, lidar, c)
		}
	})
}

func (cartoSvc *CartographerService) readData(ctx context.Context, lidar lidar.Lidar, c chan int) {
	for {
		if err := ctx.Err(); err != nil {
			if !errors.Is(err, context.Canceled) {
				cartoSvc.logger.Errorw("unexpected error in SLAM data process", "error", err)
			}
			return
		}

		cartoSvc.getNextDataPoint(ctx, lidar, c)
	}
}

func (cartoSvc *CartographerService) readDataOnInterval(ctx context.Context, lidar lidar.Lidar, c chan int) {
	ticker := time.NewTicker(time.Millisecond * time.Duration(cartoSvc.dataRateMs))
	defer ticker.Stop()
	defer cartoSvc.activeBackgroundWorkers.Done()

	for {
		if err := ctx.Err(); err != nil {
			if !errors.Is(err, context.Canceled) {
				cartoSvc.logger.Errorw("unexpected error in SLAM data process", "error", err)
			}
			return
		}

		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			if err := ctx.Err(); err != nil {
				if !errors.Is(err, context.Canceled) {
					cartoSvc.logger.Errorw("unexpected error in SLAM data process", "error", err)
				}
				return
			}

			cartoSvc.activeBackgroundWorkers.Add(1)
			goutils.PanicCapturingGo(func() {
				defer cartoSvc.activeBackgroundWorkers.Done()
				cartoSvc.getNextDataPoint(ctx, lidar, c)
			})
		}
	}
}

func (cartoSvc *CartographerService) getNextDataPoint(ctx context.Context, lidar lidar.Lidar, c chan int) {
	if _, err := dim2d.GetAndSaveData(ctx, cartoSvc.dataDirectory, lidar, cartoSvc.logger); err != nil {
		if strings.Contains(err.Error(), "reached end of dataset") {
			if !cartoSvc.jobDone {
				cartoSvc.logger.Warn(err)
				_, err = os.Create(filepath.Join(cartoSvc.dataDirectory, "job_done.txt"))
				if err != nil {
					cartoSvc.logger.Warn(err)
				}
				cartoSvc.jobDone = true
			}
		} else {
			cartoSvc.logger.Warn(err)
		}
	}
	if c != nil {
		c <- 1
	}
}

// DoCommand receives arbitrary commands.
func (cartoSvc *CartographerService) DoCommand(ctx context.Context, req map[string]interface{}) (map[string]interface{}, error) {
	if cartoSvc.closed {
		cartoSvc.logger.Warn("DoCommand called after closed")
		return nil, ErrClosed
	}

	if cartoSvc.modularizationV2Enabled {
		return cartoSvc.doCommandModularizationV2(req)
	}

	return nil, viamgrpc.UnimplementedError
}

func (cartoSvc *CartographerService) doCommandModularizationV2(
	req map[string]interface{},
) (map[string]interface{}, error) {
	if _, ok := req["job_done"]; ok {
		return map[string]interface{}{"job_done": cartoSvc.jobDone}, nil
	}

	return nil, viamgrpc.UnimplementedError
}

// Close out of all slam related processes.
func (cartoSvc *CartographerService) Close(ctx context.Context) error {
	cartoSvc.mu.Lock()
	defer cartoSvc.mu.Unlock()
	if cartoSvc.closed {
		cartoSvc.logger.Warn("Close() called multiple times")
		return nil
	}
	if cartoSvc.modularizationV2Enabled {
		// stop sensor process workers
		cartoSvc.cancelSensorProcessFunc()
		cartoSvc.sensorProcessWorkers.Wait()

		// terminate carto facade
		err := terminateCartoFacade(ctx, cartoSvc)
		if err != nil {
			cartoSvc.logger.Errorw("close hit error", "error", err)
		}

		// stop carto facade workers
		cartoSvc.cancelCartoFacadeFunc()
		cartoSvc.cartoFacadeWorkers.Wait()
		cartoSvc.closed = true
		return nil
	}

	defer func() {
		if cartoSvc.clientAlgoClose != nil {
			goutils.UncheckedErrorFunc(cartoSvc.clientAlgoClose)
		}
	}()
	cartoSvc.cancelFunc()
	if cartoSvc.bufferSLAMProcessLogs {
		if cartoSvc.slamProcessLogReader != nil {
			if err := cartoSvc.slamProcessLogReader.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log reader")
			}
		}
		if cartoSvc.slamProcessLogWriter != nil {
			if err := cartoSvc.slamProcessLogWriter.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log writer")
			}
		}
	}
	if err := cartoSvc.StopSLAMProcess(); err != nil {
		return errors.Wrap(err, "error occurred during closeout of process")
	}
	cartoSvc.activeBackgroundWorkers.Wait()
	cartoSvc.closed = true
	return nil
}

// GetSLAMProcessConfig returns the process config for the SLAM process.
func (cartoSvc *CartographerService) GetSLAMProcessConfig() pexec.ProcessConfig {
	var args []string

	args = append(args, "-sensors="+cartoSvc.primarySensorName)
	args = append(args, "-config_param="+vcUtils.DictToString(cartoSvc.configParams))
	args = append(args, "-data_rate_ms="+strconv.Itoa(cartoSvc.dataRateMs))
	args = append(args, "-map_rate_sec="+strconv.Itoa(cartoSvc.mapRateSec))
	args = append(args, "-data_dir="+cartoSvc.dataDirectory)
	args = append(args, "-delete_processed_data="+strconv.FormatBool(cartoSvc.deleteProcessedData))
	args = append(args, "-use_live_data="+strconv.FormatBool(cartoSvc.useLiveData))
	args = append(args, "-port="+cartoSvc.port)
	args = append(args, "--aix-auto-update")

	target := cartoSvc.executableName

	appDir := os.Getenv("APPDIR")

	if appDir != "" {
		// The carto grpc server is expected to be in
		// /usr/bin/ if we are running in an AppImage
		target = appDir + "/usr/bin/" + strings.TrimPrefix(target, "/")
	}

	return pexec.ProcessConfig{
		ID:   "slam_cartographer",
		Name: target,
		Args: args,
		Log:  true,
		// In appimage this is set to the appimage
		// squashfs mount location (/tmp/.mountXXXXX)
		// Otherwise, it is an empty string
		CWD:     os.Getenv("APPRUN_RUNTIME"),
		OneShot: false,
	}
}

// GetSLAMProcessBufferedLogReader is deprecated.
func (cartoSvc *CartographerService) GetSLAMProcessBufferedLogReader() bufio.Reader {
	return cartoSvc.slamProcessBufferedLogReader
}

// StartSLAMProcess starts up the SLAM library process by calling the executable binary and giving it the necessary arguments.
func (cartoSvc *CartographerService) StartSLAMProcess(ctx context.Context) error {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::slamService::StartSLAMProcess")
	defer span.End()

	processConfig := cartoSvc.GetSLAMProcessConfig()

	var logReader io.ReadCloser
	var logWriter io.WriteCloser
	var bufferedLogReader bufio.Reader
	if cartoSvc.port == localhost0 || cartoSvc.bufferSLAMProcessLogs {
		logReader, logWriter = io.Pipe()
		bufferedLogReader = *bufio.NewReader(logReader)
		processConfig.LogWriter = logWriter
	}

	_, err := cartoSvc.slamProcess.AddProcessFromConfig(ctx, processConfig)
	if err != nil {
		return errors.Wrap(err, "problem adding slam process")
	}

	cartoSvc.logger.Debug("starting slam process")

	if err = cartoSvc.slamProcess.Start(ctx); err != nil {
		return errors.Wrap(err, "problem starting slam process")
	}

	if cartoSvc.port == localhost0 {
		timeoutCtx, timeoutCancel := context.WithTimeout(ctx, parsePortMaxTimeoutSec*time.Second)
		defer timeoutCancel()

		if !cartoSvc.bufferSLAMProcessLogs {
			defer func(logger golog.Logger) {
				if err := logReader.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(cartoSvc.logger)
			defer func(logger golog.Logger) {
				if err := logWriter.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(cartoSvc.logger)
		}

		for {
			if err := timeoutCtx.Err(); err != nil {
				return errors.Wrapf(err, "error getting port from slam process")
			}

			line, err := bufferedLogReader.ReadString('\n')
			if err != nil {
				return errors.Wrapf(err, "error getting port from slam process")
			}
			portLogLinePrefix := "Server listening on "
			if strings.Contains(line, portLogLinePrefix) {
				linePieces := strings.Split(line, portLogLinePrefix)
				if len(linePieces) != 2 {
					return errors.Errorf("failed to parse port from slam process log line: %v", line)
				}
				cartoSvc.port = "localhost:" + strings.TrimRight(linePieces[1], "\n")
				break
			}
		}
	}

	if cartoSvc.bufferSLAMProcessLogs {
		cartoSvc.slamProcessLogReader = logReader
		cartoSvc.slamProcessLogWriter = logWriter
		cartoSvc.slamProcessBufferedLogReader = bufferedLogReader
	}

	return nil
}

// StopSLAMProcess uses the process manager to stop the created slam process from running.
func (cartoSvc *CartographerService) StopSLAMProcess() error {
	if err := cartoSvc.slamProcess.Stop(); err != nil {
		return errors.Wrap(err, "problem stopping slam process")
	}
	return nil
}
