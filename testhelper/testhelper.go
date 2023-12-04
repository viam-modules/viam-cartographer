// Package testhelper provides test helpers which don't depend on viamcartographer
package testhelper

import (
	"context"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	s "github.com/viamrobotics/viam-cartographer/sensors"
)

const (
	// SlamTimeFormat is the timestamp format used in the dataprocess.
	SlamTimeFormat = "2006-01-02T15:04:05.0000Z"
	// CartoFacadeTimeoutForTest is the timeout used for capi requests for tests.
	CartoFacadeTimeoutForTest = 5 * time.Second
	// CartoFacadeInternalTimeoutForTest is the timeout used for internal capi requests for tests.
	CartoFacadeInternalTimeoutForTest = 15 * time.Minute

	// LidarWithErroringFunctions is a lidar whose functions return errors.
	LidarWithErroringFunctions s.TestSensor = "stub_lidar"
	// MovementSensorWithErroringFunctions is a movement sensor whose functions return errors.
	MovementSensorWithErroringFunctions s.TestSensor = "stub_movement_sensor"
	// NoMovementSensor is a movement sensor that represents that no movement sensor is set up or added.
	NoMovementSensor s.TestSensor = ""
)

var (
	testLidars = map[s.TestSensor]func(*testing.T) *inject.Camera{
		LidarWithErroringFunctions: getLidarWithErroringFunctions,
	}

	testMovementSensors = map[s.TestSensor]func(*testing.T) *inject.MovementSensor{
		MovementSensorWithErroringFunctions: getMovementSensorWithErroringFunctions,
		NoMovementSensor:                    nil,
	}
)

// SetupStubDeps returns stubbed dependencies based on the camera
// the stubs fail tests if called.
func SetupStubDeps(lidarName, movementSensorName s.TestSensor, t *testing.T) resource.Dependencies {
	deps := make(resource.Dependencies)
	if getLidarFunc, ok := testLidars[lidarName]; ok {
		deps[camera.Named(string(lidarName))] = getLidarFunc(t)
	} else {
		t.Errorf("SetupStubDeps called with unhandled camera: %s", string(lidarName))
	}

	if getMovementSensorFunc, ok := testMovementSensors[movementSensorName]; ok {
		if getMovementSensorFunc != nil {
			deps[movementsensor.Named(string(movementSensorName))] = getMovementSensorFunc(t)
		}
	} else {
		t.Errorf("SetupStubDeps called with unhandled movement sensor: %s", string(movementSensorName))
	}

	return deps
}

func getLidarWithErroringFunctions(t *testing.T) *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		t.Error("TEST FAILED stub lidar NextPointCloud called")
		return nil, errors.New("invalid sensor")
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		t.Error("TEST FAILED stub lidar Stream called")
		return nil, errors.New("invalid sensor")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		t.Error("TEST FAILED stub lidar Projector called")
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{
			SupportsPCD: true,
		}, nil
	}
	return cam
}

func getMovementSensorWithErroringFunctions(t *testing.T) *inject.MovementSensor {
	movementSensor := &inject.MovementSensor{}
	movementSensor.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		t.Error("TEST FAILED stub movement sensor LinearAcceleration called")
		return r3.Vector{}, errors.New("invalid sensor")
	}
	movementSensor.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		t.Error("TEST FAILED stub movement sensor AngularVelocity called")
		return spatialmath.AngularVelocity{}, errors.New("invalid sensor")
	}
	movementSensor.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
		}, nil
	}
	return movementSensor
}

// ClearDirectory deletes the contents in the path directory
// without deleting path itself.
func ClearDirectory(t *testing.T, path string) {
	t.Helper()

	err := ResetFolder(path)
	test.That(t, err, test.ShouldBeNil)
}

// CreateIntegrationSLAMService creates a slam service for testing.
func CreateIntegrationSLAMService(
	t *testing.T,
	cfg *vcConfig.Config,
	timedLidar s.TimedLidar,
	timedMovementSensor s.TimedMovementSensor,
	logger logging.Logger,
) (slam.Service, error) {
	ctx := context.Background()
	cfgService := resource.Config{Name: "test", API: slam.API, Model: viamcartographer.Model}
	cfgService.ConvertedAttributes = cfg

	sensorDeps, err := cfg.Validate("path")
	if err != nil {
		return nil, err
	}
	if timedMovementSensor == nil {
		test.That(t, sensorDeps, test.ShouldResemble, []string{cfg.Camera["name"]})
	} else {
		test.That(t, sensorDeps, test.ShouldResemble, []string{cfg.Camera["name"], cfg.MovementSensor["name"]})
	}

	deps := SetupStubDeps(s.TestSensor(cfg.Camera["name"]), s.TestSensor(cfg.MovementSensor["name"]), t)

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		CartoFacadeTimeoutForTest,
		CartoFacadeInternalTimeoutForTest,
		timedLidar,
		timedMovementSensor,
	)
	if err != nil {
		test.That(t, svc, test.ShouldBeNil)
		return nil, err
	}

	test.That(t, svc, test.ShouldNotBeNil)

	return svc, nil
}

// CreateSLAMService creates a slam service for testing.
func CreateSLAMService(
	t *testing.T,
	cfg *vcConfig.Config,
	logger logging.Logger,
) (slam.Service, error) {
	t.Helper()

	ctx := context.Background()
	cfgService := resource.Config{Name: "test", API: slam.API, Model: viamcartographer.Model}
	cfgService.ConvertedAttributes = cfg
	sensorDeps, err := cfg.Validate("path")
	if err != nil {
		return nil, err
	}

	cameraName := cfg.Camera["name"]
	movementSensorName := cfg.MovementSensor["name"]

	if movementSensorName == "" {
		test.That(t, sensorDeps, test.ShouldResemble, []string{cameraName})
	} else {
		test.That(t, sensorDeps, test.ShouldResemble, []string{cameraName, movementSensorName})
	}

	deps := s.SetupDeps(s.TestSensor(cameraName), s.TestSensor(movementSensorName))

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		CartoFacadeTimeoutForTest,
		CartoFacadeInternalTimeoutForTest,
		nil,
		nil,
	)
	if err != nil {
		test.That(t, svc, test.ShouldBeNil)
		return nil, err
	}

	test.That(t, svc, test.ShouldNotBeNil)

	return svc, nil
}

// ResetFolder removes all content in path and creates a new directory
// in its place.
func ResetFolder(path string) error {
	dirInfo, err := os.Stat(path)
	if err != nil {
		return err
	}
	if !dirInfo.IsDir() {
		return errors.Errorf("the path passed ResetFolder does not point to a folder: %v", path)
	}
	if err = os.RemoveAll(path); err != nil {
		return err
	}
	return os.Mkdir(path, dirInfo.Mode())
}

// InitTestCL initializes the carto library & returns a function to terminate it.
func InitTestCL(t *testing.T, logger logging.Logger) func() {
	t.Helper()
	err := viamcartographer.InitCartoLib(logger)
	test.That(t, err, test.ShouldBeNil)
	return func() {
		err = viamcartographer.TerminateCartoLib()
		test.That(t, err, test.ShouldBeNil)
	}
}

// InitInternalState creates the internal state directory witghin a temp directory
// with an internal state pbstream file & returns the data directory & a function
// to delete the data directory.
func InitInternalState(t *testing.T) (string, func()) {
	dataDirectory, err := os.MkdirTemp("", "*")
	test.That(t, err, test.ShouldBeNil)

	internalStateDir := filepath.Join(dataDirectory, "internal_state")
	err = os.Mkdir(internalStateDir, os.ModePerm)
	test.That(t, err, test.ShouldBeNil)

	file := "viam-cartographer/outputs/viam-office-02-22-3/internal_state/internal_state_0.pbstream"
	internalState, err := os.ReadFile(artifact.MustPath(file))
	test.That(t, err, test.ShouldBeNil)

	timestamp := time.Date(2006, 1, 2, 15, 4, 5, 999900000, time.UTC)
	filename := CreateTimestampFilename(dataDirectory+"/internal_state", "internal_state", ".pbstream", timestamp)
	err = os.WriteFile(filename, internalState, os.ModePerm)
	test.That(t, err, test.ShouldBeNil)

	return filename, func() {
		err := os.RemoveAll(dataDirectory)
		test.That(t, err, test.ShouldBeNil)
	}
}

// CreateTimestampFilename creates an absolute filename with a primary sensor name and timestamp written
// into the filename.
func CreateTimestampFilename(dataDirectory, lidarName, fileType string, timeStamp time.Time) string {
	return filepath.Join(dataDirectory, lidarName+"_data_"+timeStamp.UTC().Format(SlamTimeFormat)+fileType)
}
