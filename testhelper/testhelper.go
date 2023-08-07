// Package testhelper provides test helpers which don't depend on viamcartographer
package testhelper

import (
	"bytes"
	"context"
	"fmt"
	"os"
	"path"
	"path/filepath"
	"strconv"
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
	testDialMaxTimeoutSec              = 1
	// NumPointClouds is the number of pointclouds saved in artifact
	// for the cartographer integration tests.
	NumPointClouds = 15
)

var mockLidarPath = artifact.MustPath("viam-cartographer/mock_lidar")

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
		return deps
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

func mockLidarReadingsValid() error {
	dirEntries, err := os.ReadDir(mockLidarPath)
	if err != nil {
		return err
	}

	var files []string
	for _, f := range dirEntries {
		if !f.IsDir() {
			files = append(files, f.Name())
		}
	}
	if len(files) < NumPointClouds {
		return errors.New("expected at least 15 lidar readings for integration test")
	}
	for i := 0; i < NumPointClouds; i++ {
		found := false
		expectedFile := fmt.Sprintf("%d.pcd", i)
		for _, file := range files {
			if file == expectedFile {
				found = true
				break
			}
		}

		if !found {
			return errors.Errorf("expected %s to exist for integration test", path.Join(mockLidarPath, expectedFile))
		}
	}
	return nil
}

// IntegrationTimedLidarSensor returns a mock timed lidar sensor
// or an error if preconditions to build the mock are not met.
// It validates that all required mock lidar reading files are able to be found.
// When the mock is called, it returns the next mock lidar reading, with the
// ReadingTime incremented by the sensorReadingInterval.
// The Replay sensor field of the mock readings will match the replay parameter.
// When the end of the mock lidar readings is reached, the done channel
// is written to once so the caller can detect all lidar readings have been emitted
// from the mock. This is intended to match the same "end of dataset" behavior of a
// replay sensor.
// It is important to provide deterministic time information to cartographer to
// ensure test outputs of cartographer are deterministic.
func IntegrationTimedLidarSensor(
	t *testing.T,
	lidar string,
	replay bool,
	sensorReadingInterval time.Duration,
	done chan struct{},
) (s.TimedLidarSensor, error) {
	err := mockLidarReadingsValid()
	if err != nil {
		return nil, err
	}

	var i uint64
	closed := false

	ts := &s.TimedLidarSensorMock{}
	readingTime := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)

	ts.TimedLidarSensorReadingFunc = func(ctx context.Context) (s.TimedLidarSensorReadingResponse, error) {
		readingTime = readingTime.Add(sensorReadingInterval)
		t.Logf("TimedLidarSensorReading Mock i: %d, closed: %v, readingTime: %s\n", i, closed, readingTime.String())
		if i >= NumPointClouds {
			// communicate to the test that all lidar readings have been written
			if !closed {
				done <- struct{}{}
				closed = true
			}
			return s.TimedLidarSensorReadingResponse{}, errors.New("end of dataset")
		}

		file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/" + strconv.FormatUint(i, 10) + ".pcd"))
		if err != nil {
			t.Error("TEST FAILED TimedLidarSensorReading Mock failed to open pcd file")
			return s.TimedLidarSensorReadingResponse{}, err
		}
		readingPc, err := pointcloud.ReadPCD(file)
		if err != nil {
			t.Error("TEST FAILED TimedLidarSensorReading Mock failed to read pcd")
			return s.TimedLidarSensorReadingResponse{}, err
		}

		buf := new(bytes.Buffer)
		err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary)
		if err != nil {
			t.Error("TEST FAILED TimedLidarSensorReading Mock failed to parse pcd")
			return s.TimedLidarSensorReadingResponse{}, err
		}

		i++
		return s.TimedLidarSensorReadingResponse{Reading: buf.Bytes(), ReadingTime: readingTime, Replay: replay}, nil
	}

	return ts, nil
}

// IntegrationTimedIMUSensor returns a mock timed IMU sensor.
// When the mock is called, it returns the next mock IMU readings, with the
// ReadingTime incremented by the sensorReadingInterval.
// The Replay sensor field of the mock readings will match the replay parameter.
// When the end of the mock IMU readings is reached, the done channel
// is written to once so the caller can detect all IMU readings have been emitted
// from the mock. This is intended to match the same "end of dataset" behavior of a
// replay sensor.
// It is important to provide deterministic time information to cartographer to
// ensure test outputs of cartographer are deterministic.
// Note that IMU replay sensors are not yet fully supported.
func IntegrationTimedIMUSensor(
	t *testing.T,
	imu string,
	replay bool,
	sensorReadingInterval time.Duration,
	done chan struct{},
) (s.TimedIMUSensor, error) {
	if imu == "" {
		return nil, nil
	}
	var i uint64
	closed := false

	ts := &s.TimedIMUSensorMock{}
	mockLinearAccelerationData := []r3.Vector{
		{X: 1, Y: 1, Z: 1},
		{X: 2, Y: 1, Z: 1},
		{X: 1, Y: 2, Z: 1},
		{X: 1, Y: 1, Z: 2},
		{X: 1, Y: 1, Z: 3},
		{X: 1, Y: 3, Z: 1},
		{X: 3, Y: 1, Z: 1},
		{X: 2, Y: 1, Z: 1},
	}
	mockAngularVelocityData := []spatialmath.AngularVelocity{
		{X: 1, Y: 2, Z: 1},
		{X: 1, Y: 2, Z: 1},
		{X: 1, Y: 2, Z: 1},
		{X: 1, Y: 1, Z: 2},
		{X: 1, Y: 1, Z: 2},
		{X: 1, Y: 1, Z: 2},
		{X: 5, Y: 1, Z: 1},
		{X: 5, Y: 1, Z: 1},
	}

	readingTime := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)

	ts.TimedIMUSensorReadingFunc = func(ctx context.Context) (s.TimedIMUSensorReadingResponse, error) {
		readingTime = readingTime.Add(sensorReadingInterval)
		t.Logf("TimedIMUSensorReading Mock i: %d, closed: %v, readingTime: %s\n", i, closed, readingTime.String())
		if int(i) >= len(mockLinearAccelerationData) {
			// communicate to the test that all imu readings have been written
			if !closed {
				done <- struct{}{}
				closed = true
			}
			return s.TimedIMUSensorReadingResponse{}, errors.New("end of dataset")
		}
		linAcc := mockLinearAccelerationData[i]
		angVel := mockAngularVelocityData[i]
		i++
		return s.TimedIMUSensorReadingResponse{LinearAcceleration: linAcc, AngularVelocity: angVel, ReadingTime: readingTime, Replay: replay}, nil
	}

	return ts, nil
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
	test.That(t, sensorDeps, test.ShouldResemble, []string{cfg.Camera["name"]})
	deps := SetupStubDeps(cfg.Camera["name"], cfg.MovementSensor["name"], t)

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		SensorValidationMaxTimeoutSecForTest,
		SensorValidationIntervalSecForTest,
		5*time.Second,
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
	lidarDeps, err := cfg.Validate("path")
	if err != nil {
		return nil, err
	}

	// feature flag for IMU Integration
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
	test.That(t, lidarDeps, test.ShouldResemble, []string{cameraName})
	deps := s.SetupDeps(cameraName, imuName)

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		SensorValidationMaxTimeoutSecForTest,
		SensorValidationIntervalSecForTest,
		5*time.Second,
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
