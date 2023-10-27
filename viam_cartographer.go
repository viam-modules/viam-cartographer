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

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.uber.org/zap/zapcore"
	viamgrpc "go.viam.com/rdk/grpc"
	"go.viam.com/rdk/logging"
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
	defaultLidarDataFrequencyHz          = 5
	defaultMovementSensorDataFrequencyHz = 20
	defaultDialMaxTimeoutSec             = 30
	defaultCartoFacadeTimeout            = 5 * time.Minute
	defaultCartoFacadeInternalTimeout    = 15 * time.Minute
	chunkSizeBytes                       = 1 * 1024 * 1024
)

var defaultCartoAlgoCfg = cartofacade.CartoAlgoConfig{
	OptimizeOnStart:      false,
	OptimizeEveryNNodes:  3,
	NumRangeData:         30,
	MissingDataRayLength: 25.0,
	MaxRange:             25.0,
	MinRange:             0.2,
	UseIMUData:           false,
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
			logger logging.Logger,
		) (slam.Service, error) {
			return New(
				ctx,
				deps,
				c,
				logger,
				defaultCartoFacadeTimeout,
				defaultCartoFacadeInternalTimeout,
				nil,
				nil,
				nil,
			)
		},
	})
}

// InitCartoLib is run to initialize the cartographer library
// must be called before module.AddModelFromRegistry is
// called.
func InitCartoLib(logger logging.Logger) error {
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

func initSensorProcesses(cancelCtx context.Context, cartoSvc *CartographerService) {
	spConfig := sensorprocess.Config{
		CartoFacade:              cartoSvc.cartofacade,
		Lidar:                    cartoSvc.lidar,
		IMU:                      cartoSvc.movementSensor,
		Timeout:                  cartoSvc.cartoFacadeTimeout,
		InternalTimeout:          cartoSvc.cartoFacadeInternalTimeout,
		Logger:                   cartoSvc.logger,
		RunFinalOptimizationFunc: cartoSvc.cartofacade.RunFinalOptimization,
		Mutex:                    &sync.Mutex{},
	}

	cartoSvc.sensorProcessWorkers.Add(1)
	go func() {
		defer cartoSvc.sensorProcessWorkers.Done()
		if jobDone := spConfig.StartLidar(cancelCtx); jobDone {
			cartoSvc.jobDone.Store(true)
			cartoSvc.cancelSensorProcessFunc()
		}
	}()

	if spConfig.IMU != nil {
		cartoSvc.sensorProcessWorkers.Add(1)
		go func() {
			defer cartoSvc.sensorProcessWorkers.Done()
			_ = spConfig.StartIMU(cancelCtx)
		}()
	}
}

// New returns a new slam service for the given robot.
func New(
	ctx context.Context,
	deps resource.Dependencies,
	c resource.Config,
	logger logging.Logger,
	cartoFacadeTimeout time.Duration,
	cartoFacadeInternalTimeout time.Duration,
	testTimedLidarOverride s.TimedLidar,
	testTimedMovementSensorOverride s.TimedMovementSensor,
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

	optionalConfigParams, err := vcConfig.GetOptionalParameters(
		svcConfig,
		defaultLidarDataFrequencyHz,
		defaultMovementSensorDataFrequencyHz,
		logger,
	)
	if err != nil {
		return nil, err
	}

	lidarName := svcConfig.Camera["name"]

	movementSensorName := optionalConfigParams.MovementSensorName
	if movementSensorName != "" {
		if optionalConfigParams.LidarDataFrequencyHz == 0 && optionalConfigParams.MovementSensorDataFrequencyHz != 0 {
			return nil, errors.New("In offline mode, but movement sensor data frequency is nonzero")
		}

		if optionalConfigParams.LidarDataFrequencyHz != 0 && optionalConfigParams.MovementSensorDataFrequencyHz == 0 {
			return nil, errors.New("In online mode, but movement sensor data frequency is zero")
		}
	}

	// Get the lidar for the Dim2D cartographer sub algorithm
	timedLidar, err := s.NewLidar(ctx, deps, lidarName, optionalConfigParams.LidarDataFrequencyHz, logger)
	if err != nil {
		return nil, err
	}

	// Get the movement sensor if one is configured and check if it supports an IMU and/or odometer.
	var timedMovementSensor s.TimedMovementSensor
	if movementSensorName == "" {
		logger.Info("no movement sensor configured, proceeding without IMU and without odometer")
	} else if timedMovementSensor, err = s.NewMovementSensor(ctx, deps, movementSensorName,
		optionalConfigParams.MovementSensorDataFrequencyHz, logger); err != nil {
		return nil, err
	}

	// Need to be able to shut down the sensor process before the cartoFacade
	cancelSensorProcessCtx, cancelSensorProcessFunc := context.WithCancel(context.Background())
	cancelCartoFacadeCtx, cancelCartoFacadeFunc := context.WithCancel(context.Background())

	// Override the sensors for testing if the override sensors are not nil
	if testTimedLidarOverride != nil {
		timedLidar = testTimedLidarOverride
	}
	if testTimedMovementSensorOverride != nil {
		timedMovementSensor = testTimedMovementSensorOverride
	}
	if testTimedOdometerSensorOverride != nil {
		timedOdometer = testTimedOdometerSensorOverride
	}

	// Cartographer SLAM Service Object
	cartoSvc := &CartographerService{
		Named:                      c.ResourceName().AsNamed(),
		lidar:                      timedLidar,
		movementSensor:             timedMovementSensor,
		movementSensorName:         movementSensorName,
		subAlgo:                    subAlgo,
		configParams:               svcConfig.ConfigParams,
		cancelSensorProcessFunc:    cancelSensorProcessFunc,
		cancelCartoFacadeFunc:      cancelCartoFacadeFunc,
		logger:                     logger,
		cartoFacadeTimeout:         cartoFacadeTimeout,
		cartoFacadeInternalTimeout: cartoFacadeInternalTimeout,
		mapTimestamp:               time.Now().UTC(),
		enableMapping:              optionalConfigParams.EnableMapping,
		existingMap:                optionalConfigParams.ExistingMap,
	}

	defer func() {
		if err != nil {
			logger.Errorw("New() hit error, closing...", "error", err)
			if err := cartoSvc.Close(ctx); err != nil {
				logger.Errorw("error closing out after error", "error", err)
			}
		}
	}()

	// do not initialize CartoFacade or Sensor Processes when using cloudslam
	if svcConfig.UseCloudSlam != nil && *svcConfig.UseCloudSlam {
		return &CartographerService{
			Named:        c.ResourceName().AsNamed(),
			useCloudSlam: true,
			logger:       logger,
		}, nil
	}
	if cartoSvc.odometer != nil {
		if err = s.ValidateGetOdometerData(
			cancelSensorProcessCtx,
			timedOdometer,
			time.Duration(sensorValidationMaxTimeoutSec)*time.Second,
			time.Duration(cartoSvc.sensorValidationIntervalSec)*time.Second,
			cartoSvc.logger); err != nil {
			err = errors.Wrap(err, "failed to get data from odometer")
			return nil, err
		}
	}

	if err = initCartoFacade(cancelCartoFacadeCtx, cartoSvc); err != nil {
		return nil, err
	}

	initSensorProcesses(cancelSensorProcessCtx, cartoSvc)

	return cartoSvc, nil
}

func parseCartoAlgoConfig(configParams map[string]string, logger logging.Logger) (cartofacade.CartoAlgoConfig, error) {
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
		Camera:             cartoSvc.lidar.Name(),
		MovementSensor:     cartoSvc.movementSensorName,
		ComponentReference: cartoSvc.lidar.Name(),
		LidarConfig:        cartofacade.TwoD,
		EnableMapping:      cartoSvc.enableMapping,
		ExistingMap:        cartoSvc.existingMap,
	}

	if cartoSvc.movementSensorName == "" {
		cartoSvc.logger.Debug("No movement sensor provided, setting use_imu_data to false")
	} else {
		movementSensorProperties := cartoSvc.movementSensor.Properties()
		if movementSensorProperties.IMUSupported {
			cartoSvc.logger.Warn("IMU configured, setting use_imu_data to true")
			cartoAlgoConfig.UseIMUData = true
		} else {
			cartoSvc.logger.Warn("Movement sensor was provided but does not support IMU data, setting use_imu_data to false")
		}
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

// CartographerService is the structure of the slam service.
type CartographerService struct {
	resource.Named
	resource.AlwaysRebuild
	mu                 sync.Mutex
	SlamMode           cartofacade.SlamMode
	closed             bool
	lidar              s.TimedLidar
	movementSensor     s.TimedMovementSensor
	movementSensorName string
	subAlgo            SubAlgo

	configParams map[string]string

	cartofacade                cartofacade.Interface
	cartoFacadeTimeout         time.Duration
	cartoFacadeInternalTimeout time.Duration

	cancelSensorProcessFunc func()
	cancelCartoFacadeFunc   func()
	logger                  logging.Logger
	sensorProcessWorkers    sync.WaitGroup
	cartoFacadeWorkers      sync.WaitGroup

	mapTimestamp time.Time
	jobDone      atomic.Bool

	useCloudSlam  bool
	enableMapping bool
	existingMap   string
}

// Position forwards the request for positional data to the slam library's gRPC service. Once a response is received,
// it is unpacked into a Pose and a component reference string.
func (cartoSvc *CartographerService) Position(ctx context.Context) (spatialmath.Pose, string, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::Position")
	defer span.End()
	if cartoSvc.useCloudSlam {
		cartoSvc.logger.Warn("Position called with use_cloud_slam set to true")
		return nil, "", ErrUseCloudSlamEnabled
	}
	if cartoSvc.closed {
		cartoSvc.logger.Warn("Position called after closed")
		return nil, "", ErrClosed
	}

	pos, err := cartoSvc.cartofacade.Position(ctx, cartoSvc.cartoFacadeTimeout)
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
	return CheckQuaternionFromClientAlgo(pose, cartoSvc.lidar.Name(), returnedExt)
}

// PointCloudMap creates a request calls the slam algorithms PointCloudMap endpoint and returns a callback
// function which will return the next chunk of the current pointcloud map.
func (cartoSvc *CartographerService) PointCloudMap(ctx context.Context) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::PointCloudMap")
	defer span.End()
	if cartoSvc.useCloudSlam {
		cartoSvc.logger.Warn("PointCloudMap called with use_cloud_slam set to true")
		return nil, ErrUseCloudSlamEnabled
	}

	if cartoSvc.closed {
		cartoSvc.logger.Warn("PointCloudMap called after closed")
		return nil, ErrClosed
	}

	pc, err := cartoSvc.cartofacade.PointCloudMap(ctx, cartoSvc.cartoFacadeTimeout)
	if err != nil {
		return nil, err
	}
	return toChunkedFunc(pc), nil
}

// InternalState creates a request, calls the slam algorithms InternalState endpoint and returns a callback
// function which will return the next chunk of the current internal state of the slam algo.
func (cartoSvc *CartographerService) InternalState(ctx context.Context) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::InternalState")
	defer span.End()
	if cartoSvc.useCloudSlam {
		cartoSvc.logger.Warn("InternalState called with use_cloud_slam set to true")
		return nil, ErrUseCloudSlamEnabled
	}

	if cartoSvc.closed {
		cartoSvc.logger.Warn("InternalState called after closed")
		return nil, ErrClosed
	}

	is, err := cartoSvc.cartofacade.InternalState(ctx, cartoSvc.cartoFacadeTimeout)
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

// LatestMapInfo returns a new timestamp every time it is called when in mapping mode, to signal
// that the map should be updated. In localizing, the timestamp returned is the timestamp of the session.
func (cartoSvc *CartographerService) LatestMapInfo(ctx context.Context) (time.Time, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::CartographerService::LatestMapInfo")
	defer span.End()
	if cartoSvc.useCloudSlam {
		cartoSvc.logger.Warn("LatestMapInfo called with use_cloud_slam set to true")
		return time.Time{}, ErrUseCloudSlamEnabled
	}

	if cartoSvc.closed {
		cartoSvc.logger.Warn("LatestMapInfo called after closed")
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

	cartoSvc.logger.Info("Closing cartographer module")

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

	cartoSvc.logger.Info("Closing complete")
	return nil
}

// CheckQuaternionFromClientAlgo checks to see if the internal SLAM algorithm sent a quaternion. If it did,
// return the updated pose.
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
