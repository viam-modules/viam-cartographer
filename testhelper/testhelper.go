// Package testhelper provides test helpers which don't depend on viamcartographer
package testhelper

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"github.com/viamrobotics/gostream"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/movementsensor"
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
	// SensorValidationMaxTimeoutSecForTest is used in the ValidateGetAndSaveData
	// function to ensure that the sensor in the GetAndSaveData function
	// returns data within an acceptable time.
	SensorValidationMaxTimeoutSecForTest = 1
	// SensorValidationIntervalSecForTest is used in the ValidateGetAndSaveData
	// function for the while loop that attempts to grab data from the
	// sensor that is used in the GetAndSaveData function.
	SensorValidationIntervalSecForTest = 1
	// CartoFacadeTimeoutForTest is the timeout used for capi requests for tests.
	CartoFacadeTimeoutForTest = 5 * time.Second
	// CartoFacadeInternalTimeoutForTest is the timeout used for internal capi requests for tests.
	CartoFacadeInternalTimeoutForTest = 15 * time.Minute
)

// SetupStubDeps returns stubbed dependencies based on the camera
// the stubs fail tests if called.
func SetupStubDeps(cameraName, movementSensorName string, t *testing.T) resource.Dependencies {
	deps := make(resource.Dependencies)
	switch cameraName {
	case "stub_lidar":
		deps[camera.Named(cameraName)] = getStubLidar(t)
	default:
		t.Errorf("SetupStubDeps called with unhandled camera: %s", cameraName)
	}
	switch movementSensorName {
	case "stub_imu":
		deps[movementsensor.Named(movementSensorName)] = getStubIMU(t)
	case "":
	default:
		t.Errorf("SetupStubDeps called with unhandled movement sensor: %s", movementSensorName)
	}

	return deps
}

func getStubLidar(t *testing.T) *inject.Camera {
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

func getStubIMU(t *testing.T) *inject.MovementSensor {
	imu := &inject.MovementSensor{}
	imu.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		t.Error("TEST FAILED stub IMU LinearAcceleration called")
		return r3.Vector{}, errors.New("invalid sensor")
	}
	imu.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		t.Error("TEST FAILED stub IMU AngularVelocity called")
		return spatialmath.AngularVelocity{}, errors.New("invalid sensor")
	}
	imu.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
		}, nil
	}
	return imu
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
	timedLidar s.TimedLidarSensor,
	timedIMU s.TimedIMUSensor,
	logger golog.Logger,
) (slam.Service, error) {
	ctx := context.Background()
	cfgService := resource.Config{Name: "test", API: slam.API, Model: viamcartographer.Model}
	cfgService.ConvertedAttributes = cfg

	sensorDeps, err := cfg.Validate("path")
	if err != nil {
		return nil, err
	}
	if timedIMU == nil {
		test.That(t, sensorDeps, test.ShouldResemble, []string{cfg.Camera["name"]})
	} else {
		test.That(t, sensorDeps, test.ShouldResemble, []string{cfg.Camera["name"], cfg.MovementSensor["name"]})
	}

	deps := SetupStubDeps(cfg.Camera["name"], cfg.MovementSensor["name"], t)

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		SensorValidationMaxTimeoutSecForTest,
		SensorValidationIntervalSecForTest,
		CartoFacadeTimeoutForTest,
		CartoFacadeInternalTimeoutForTest,
		timedLidar,
		timedIMU,
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
	logger golog.Logger,
) (slam.Service, error) {
	t.Helper()

	ctx := context.Background()
	cfgService := resource.Config{Name: "test", API: slam.API, Model: viamcartographer.Model}
	cfgService.ConvertedAttributes = cfg
	sensorDeps, err := cfg.Validate("path")
	if err != nil {
		return nil, err
	}

	// feature flag for IMU Integration sets whether to use the dictionary or list format for configuring sensors
	cameraName := ""
	imuName := ""
	if cfg.IMUIntegrationEnabled {
		cameraName = cfg.Camera["name"]
		imuName = cfg.MovementSensor["name"]
	} else {
		if len(cfg.Sensors) > 1 {
			return nil, errors.Errorf("configuring lidar camera error: "+
				"'sensors' must contain only one lidar camera, but is 'sensors: [%v]'",
				strings.Join(cfg.Sensors, ", "))
		}
		cameraName = cfg.Sensors[0]
	}
	if imuName == "" {
		test.That(t, sensorDeps, test.ShouldResemble, []string{cameraName})
	} else {
		test.That(t, sensorDeps, test.ShouldResemble, []string{cameraName, imuName})
	}

	deps := s.SetupDeps(cameraName, imuName)

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		SensorValidationMaxTimeoutSecForTest,
		SensorValidationIntervalSecForTest,
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
func InitTestCL(t *testing.T, logger golog.Logger) func() {
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

	return dataDirectory, func() {
		err := os.RemoveAll(dataDirectory)
		test.That(t, err, test.ShouldBeNil)
	}
}

// CreateTimestampFilename creates an absolute filename with a primary sensor name and timestamp written
// into the filename.
func CreateTimestampFilename(dataDirectory, lidarName, fileType string, timeStamp time.Time) string {
	return filepath.Join(dataDirectory, lidarName+"_data_"+timeStamp.UTC().Format(SlamTimeFormat)+fileType)
}
