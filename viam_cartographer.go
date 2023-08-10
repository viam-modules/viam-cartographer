// Package viamcartographer implements simultaneous localization and mapping.
// This is an Experimental package.
package viamcartographer

import (
	"bytes"
	"context"
	"strconv"
	"sync"
	"sync/atomic"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.uber.org/zap/zapcore"
	viamgrpc "go.viam.com/rdk/grpc"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	"github.com/viamrobotics/viam-cartographer/sensorprocess"
	s "github.com/viamrobotics/viam-cartographer/sensors"
)

// Model is the model name of cartographer.
var (
	Model    = resource.NewModel("viam", "slam", "cartographer")
	cartoLib cartofacade.CartoLib
	// ErrClosed denotes that the slam service method was called on a closed slam resource.
	ErrClosed = errors.Errorf("resource (%s) is closed", Model.String())
	// ErrUseCloudSlamEnabled denotes that the slam service method was called while use_cloud_slam was set to true.
	ErrUseCloudSlamEnabled = errors.Errorf("resource (%s) unavailable, configured with use_cloud_slam set to true", Model.String())
)

const (
	// DefaultExecutableName is what this program expects to call to start the cartographer grpc server.
	DefaultExecutableName                = "carto_grpc_server"
	defaultLidarDataRateMsec             = 200
	defaultIMUDataRateMsec               = 50
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
				defaultSensorValidationMaxTimeoutSec,
				defaultSensorValidationIntervalSec,
				defaultCartoFacadeTimeout,
				nil,
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
		CartoFacade:       cartoSvc.cartofacade,
		Lidar:             cartoSvc.lidar.testing,
		LidarName:         cartoSvc.lidar.name,
		LidarDataRateMsec: cartoSvc.lidar.dataRateMsec,
		Timeout:           cartoSvc.cartoFacadeTimeout,
		Logger:            cartoSvc.logger,
	}

	cartoSvc.sensorProcessWorkers.Add(1)
	go func() {
		defer cartoSvc.sensorProcessWorkers.Done()
		if jobDone := sensorprocess.Start(cancelCtx, spConfig); jobDone {
			cartoSvc.jobDone.Store(true)
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
	sensorValidationMaxTimeoutSec int,
	sensorValidationIntervalSec int,
	cartoFacadeTimeout time.Duration,
	testTimedLidarSensorOverride s.TimedLidarSensor,
	testTimedIMUSensorOverride s.TimedIMUSensor,
) (slam.Service, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::slamService::New")
	defer span.End()

	svcConfig, err := resource.NativeConfig[*vcConfig.Config](c)
	if err != nil {
		return nil, err
	}

	if svcConfig.UseCloudSlam != nil && *svcConfig.UseCloudSlam {
		return &CartographerService{
			Named:        c.ResourceName().AsNamed(),
			useCloudSlam: true,
			logger:       logger,
		}, nil
	}

	subAlgo := SubAlgo(svcConfig.ConfigParams["mode"])
	if subAlgo != Dim2d {
		return nil, errors.Errorf("%v does not have a 'mode: %v'",
			c.Model.Name, svcConfig.ConfigParams["mode"])
	}

	optionalConfigParams, err := vcConfig.GetOptionalParameters(
		svcConfig,
		defaultLidarDataRateMsec,
		defaultIMUDataRateMsec,
		defaultMapRateSec,
		logger,
	)
	if err != nil {
		return nil, err
	}

	// feature flag for new config
	lidarName := ""
	if svcConfig.IMUIntegrationEnabled {
		lidarName = svcConfig.Camera["name"]
	} else {
		lidarName = svcConfig.Sensors[0]
	}

	// Get the lidar for the Dim2D cartographer sub algorithm
	lidarObject, err := s.NewLidar(ctx, deps, lidarName, logger)
	if err != nil {
		return nil, err
	}

	// Get the IMU if one is configured
	imuObject, err := s.NewIMU(ctx, deps, optionalConfigParams.ImuName, logger)
	if err != nil {
		return nil, err
	}

	// Need to be able to shut down the sensor process before the cartoFacade
	cancelSensorProcessCtx, cancelSensorProcessFunc := context.WithCancel(context.Background())
	cancelCartoFacadeCtx, cancelCartoFacadeFunc := context.WithCancel(context.Background())

	// use the override in testing if non nil
	// otherwise use the sensors from deps as the
	// timed sensors
	timedLidar := testTimedLidarSensorOverride
	timedIMU := testTimedIMUSensorOverride
	if timedLidar == nil {
		timedLidar = lidarObject
	}
	if timedIMU == nil {
		timedIMU = imuObject
	}

	lidar := Lidar{
		name:         lidarName,
		dataRateMsec: optionalConfigParams.LidarDataRateMsec,
		actual:       lidarObject,
		testing:      timedLidar,
	}

	imu := IMU{
		name:         optionalConfigParams.ImuName,
		dataRateMsec: optionalConfigParams.ImuDataRateMsec,
		actual:       imuObject,
		testing:      timedIMU,
	}

	// Cartographer SLAM Service Object
	cartoSvc := &CartographerService{
		Named:                         c.ResourceName().AsNamed(),
		lidar:                         lidar,
		imu:                           imu,
		subAlgo:                       subAlgo,
		configParams:                  svcConfig.ConfigParams,
		dataDirectory:                 svcConfig.DataDirectory,
		mapRateSec:                    optionalConfigParams.MapRateSec,
		cancelSensorProcessFunc:       cancelSensorProcessFunc,
		cancelCartoFacadeFunc:         cancelCartoFacadeFunc,
		logger:                        logger,
		sensorValidationMaxTimeoutSec: sensorValidationMaxTimeoutSec,
		sensorValidationIntervalSec:   sensorValidationMaxTimeoutSec,
		cartoFacadeTimeout:            cartoFacadeTimeout,
		mapTimestamp:                  time.Now().UTC(),
		cloudStoryEnabled:             svcConfig.CloudStoryEnabled,
		enableMapping:                 optionalConfigParams.EnableMapping,
	}

	defer func() {
		if err != nil {
			logger.Errorw("New() hit error, closing...", "error", err)
			if err := cartoSvc.Close(ctx); err != nil {
				logger.Errorw("error closing out after error", "error", err)
			}
		}
	}()
	if err = s.ValidateGetLidarData(
		cancelSensorProcessCtx,
		timedLidar,
		time.Duration(sensorValidationMaxTimeoutSec)*time.Second,
		time.Duration(cartoSvc.sensorValidationIntervalSec)*time.Second,
		cartoSvc.logger); err != nil {
		err = errors.Wrap(err, "failed to get data from lidar")
		return nil, err
	}
	if cartoSvc.imu.name != "" {
		if err = s.ValidateGetIMUData(
			cancelSensorProcessCtx,
			timedIMU,
			time.Duration(sensorValidationMaxTimeoutSec)*time.Second,
			time.Duration(cartoSvc.sensorValidationIntervalSec)*time.Second,
			cartoSvc.logger); err != nil {
			err = errors.Wrap(err, "failed to get data from IMU")
			return nil, err
		}
	}

	err = initCartoFacade(cancelCartoFacadeCtx, cartoSvc)
	if err != nil {
		return nil, err
	}
	initSensorProcess(cancelSensorProcessCtx, cartoSvc)

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
		case "missing_data_ray_length_meters":
			fVal, err := strconv.ParseFloat(val, 32)
			if err != nil {
				return cartoAlgoCfg, err
			}
			cartoAlgoCfg.MissingDataRayLength = float32(fVal)
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
		case "max_range_meters":
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
		case "min_range_meters":
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
		case "min_covered_area_meters_squared":
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
		Camera:             cartoSvc.lidar.name,
		MovementSensor:     cartoSvc.imu.name,
		MapRateSecond:      cartoSvc.mapRateSec,
		DataDir:            cartoSvc.dataDirectory,
		ComponentReference: cartoSvc.lidar.name,
		LidarConfig:        cartofacade.TwoD,
		CloudStoryEnabled:  cartoSvc.cloudStoryEnabled,
		EnableMapping:      cartoSvc.enableMapping,
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

// Lidar is the structure containing all field related to lidar.
type Lidar struct {
	name         string
	dataRateMsec int
	actual       s.Lidar
	testing      s.TimedLidarSensor
}

// IMU is the structure containing all fields related to IMU.
type IMU struct {
	name         string
	dataRateMsec int
	actual       s.IMU
	testing      s.TimedIMUSensor
}

// CartographerService is the structure of the slam service.
type CartographerService struct {
	resource.Named
	resource.AlwaysRebuild
	mu       sync.Mutex
	SlamMode cartofacade.SlamMode
	closed   bool
	lidar    Lidar
	imu      IMU
	subAlgo  SubAlgo

	useCloudSlam bool

	configParams  map[string]string
	dataDirectory string

	cartofacade        cartofacade.Interface
	cartoFacadeTimeout time.Duration

	mapRateSec int

	cancelSensorProcessFunc func()
	cancelCartoFacadeFunc   func()
	logger                  golog.Logger
	sensorProcessWorkers    sync.WaitGroup
	cartoFacadeWorkers      sync.WaitGroup

	mapTimestamp                  time.Time
	sensorValidationMaxTimeoutSec int
	sensorValidationIntervalSec   int
	jobDone                       atomic.Bool

	cloudStoryEnabled bool
	enableMapping     bool
}

// GetPosition forwards the request for positional data to the slam library's gRPC service. Once a response is received,
// it is unpacked into a Pose and a component reference string.
func (cartoSvc *CartographerService) GetPosition(ctx context.Context) (spatialmath.Pose, string, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::GetPosition")
	defer span.End()
	if cartoSvc.useCloudSlam {
		cartoSvc.logger.Warn("GetPosition called with use_cloud_slam set to true")
		return nil, "", ErrUseCloudSlamEnabled
	}
	if cartoSvc.closed {
		cartoSvc.logger.Warn("GetPosition called after closed")
		return nil, "", ErrClosed
	}

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
	return CheckQuaternionFromClientAlgo(pose, cartoSvc.lidar.name, returnedExt)
}

// GetPointCloudMap creates a request calls the slam algorithms GetPointCloudMap endpoint and returns a callback
// function which will return the next chunk of the current pointcloud map.
func (cartoSvc *CartographerService) GetPointCloudMap(ctx context.Context) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::GetPointCloudMap")
	defer span.End()
	if cartoSvc.useCloudSlam {
		cartoSvc.logger.Warn("GetPointCloudMap called with use_cloud_slam set to true")
		return nil, ErrUseCloudSlamEnabled
	}

	if cartoSvc.closed {
		cartoSvc.logger.Warn("GetPointCloudMap called after closed")
		return nil, ErrClosed
	}

	pc, err := cartoSvc.cartofacade.GetPointCloudMap(ctx, cartoSvc.cartoFacadeTimeout)
	if err != nil {
		return nil, err
	}
	return toChunkedFunc(pc), nil
}

// GetInternalState creates a request, calls the slam algorithms GetInternalState endpoint and returns a callback
// function which will return the next chunk of the current internal state of the slam algo.
func (cartoSvc *CartographerService) GetInternalState(ctx context.Context) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::GetInternalState")
	defer span.End()
	if cartoSvc.useCloudSlam {
		cartoSvc.logger.Warn("GetInternalState called with use_cloud_slam set to true")
		return nil, ErrUseCloudSlamEnabled
	}

	if cartoSvc.closed {
		cartoSvc.logger.Warn("GetInternalState called after closed")
		return nil, ErrClosed
	}

	is, err := cartoSvc.cartofacade.GetInternalState(ctx, cartoSvc.cartoFacadeTimeout)
	if err != nil {
		return nil, err
	}

	return toChunkedFunc(is), nil
}

func toChunkedFunc(b []byte) func() ([]byte, error) {
	chunk := make([]byte, chunkSizeBytes)

	reader := bytes.NewReader(b)

	f := func() ([]byte, error) {
		bytesRead, err := reader.Read(chunk)
		if err != nil {
			return nil, err
		}
		return chunk[:bytesRead], err
	}
	return f
}

// GetLatestMapInfo returns a new timestamp every time it is called when in mapping mode, to signal
// that the map should be updated. In localizing, the timestamp returned is the timestamp of the session.
func (cartoSvc *CartographerService) GetLatestMapInfo(ctx context.Context) (time.Time, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::GetLatestMapInfo")
	defer span.End()
	if cartoSvc.useCloudSlam {
		cartoSvc.logger.Warn("GetLatestMapInfo called with use_cloud_slam set to true")
		return time.Time{}, ErrUseCloudSlamEnabled
	}

	if cartoSvc.closed {
		cartoSvc.logger.Warn("GetLatestMapInfo called after closed")
		return time.Time{}, ErrClosed
	}

	if cartoSvc.SlamMode != cartofacade.LocalizingMode {
		cartoSvc.mapTimestamp = time.Now().UTC()
	}

	return cartoSvc.mapTimestamp, nil
}

// DoCommand receives arbitrary commands.
func (cartoSvc *CartographerService) DoCommand(ctx context.Context, req map[string]interface{}) (map[string]interface{}, error) {
	if cartoSvc.useCloudSlam {
		cartoSvc.logger.Warn("DoCommand called with use_cloud_slam set to true")
		return nil, ErrUseCloudSlamEnabled
	}
	if cartoSvc.closed {
		cartoSvc.logger.Warn("DoCommand called after closed")
		return nil, ErrClosed
	}

	if _, ok := req["job_done"]; ok {
		return map[string]interface{}{"job_done": cartoSvc.jobDone.Load()}, nil
	}

	return nil, viamgrpc.UnimplementedError
}

// Close out of all slam related processes.
func (cartoSvc *CartographerService) Close(ctx context.Context) error {
	cartoSvc.mu.Lock()
	if cartoSvc.useCloudSlam {
		return nil
	}

	defer cartoSvc.mu.Unlock()
	if cartoSvc.closed {
		cartoSvc.logger.Warn("Close() called multiple times")
		return nil
	}
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

// CheckQuaternionFromClientAlgo checks to see if the internal SLAM algorithm sent a quaternion. If it did, return the updated pose.
func CheckQuaternionFromClientAlgo(pose spatialmath.Pose, componentReference string,
	returnedExt map[string]interface{},
) (spatialmath.Pose, string, error) {
	// check if extra contains a quaternion. If no quaternion is found, throw an error
	if val, ok := returnedExt["quat"]; ok {
		q := val.(map[string]interface{})

		valReal, ok1 := q["real"].(float64)
		valIMag, ok2 := q["imag"].(float64)
		valJMag, ok3 := q["jmag"].(float64)
		valKMag, ok4 := q["kmag"].(float64)

		if !ok1 || !ok2 || !ok3 || !ok4 {
			return nil, "", errors.Errorf("error getting SLAM position: quaternion given, but invalid format detected, %v", q)
		}
		actualPose := spatialmath.NewPose(pose.Point(),
			&spatialmath.Quaternion{Real: valReal, Imag: valIMag, Jmag: valJMag, Kmag: valKMag})
		return actualPose, componentReference, nil
	}
	return nil, "", errors.Errorf("error getting SLAM position: quaternion not given, %v", returnedExt)
}
