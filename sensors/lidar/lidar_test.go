package lidar_test

import (
	"context"
	"errors"
	"image/color"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"

	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
	dim2d "github.com/viamrobotics/viam-cartographer/sensors/lidar/dim-2d"
	"github.com/viamrobotics/viam-cartographer/testhelper"
)

const (
	testLidarName  = "testLidarName"
	wrongLidarName = "wrongLidarName"
)

func TestNew(t *testing.T) {
	t.Run("Empty sensors array failure", func(t *testing.T) {
		cam := &inject.Camera{}
		deps := make(resource.Dependencies)
		deps[camera.Named(testLidarName)] = cam
		sensors := []string{}
		sensorIndex := 0

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldEqual, "index out of bounds")
		test.That(t, testLidar, test.ShouldResemble, lidar.Lidar{})
	})

	t.Run("Negative sensor index failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{testLidarName}
		sensorIndex := -1

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldEqual, "index out of bounds")
		test.That(t, testLidar, test.ShouldResemble, lidar.Lidar{})
	})

	t.Run("Sensor index out of sensor array bounds failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{testLidarName}
		sensorIndex := 1

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldEqual, "index out of bounds")
		test.That(t, testLidar, test.ShouldResemble, lidar.Lidar{})
	})

	t.Run("Empty dependencies empty sensors failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{}
		sensorIndex := 0

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldEqual, "index out of bounds")
		test.That(t, testLidar, test.ShouldResemble, lidar.Lidar{})
	})

	t.Run("Empty dependencies non-empty sensors failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{testLidarName}
		sensorIndex := 0
		expectedErrorMessage := "error getting lidar camera " + testLidarName + " for slam service"

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, expectedErrorMessage)
		test.That(t, testLidar, test.ShouldResemble, lidar.Lidar{})
	})

	t.Run("Wrong sensor name failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{wrongLidarName}
		sensorIndex := 0
		expectedErrorMessage := "error getting lidar camera " + wrongLidarName + " for slam service"

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, expectedErrorMessage)
		test.That(t, testLidar, test.ShouldResemble, lidar.Lidar{})
	})

	t.Run("Successful creation of Lidar", func(t *testing.T) {
		cam := &inject.Camera{}
		deps := make(resource.Dependencies)
		deps[camera.Named(testLidarName)] = cam
		sensors := []string{testLidarName}
		sensorIndex := 0

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, testLidar, test.ShouldNotBeNil)
		test.That(t, testLidar.Name, test.ShouldEqual, testLidarName)
	})
}

func TestGetData(t *testing.T) {
	t.Run("NextPointCloud not implemented failure", func(t *testing.T) {
		expectedErrorMessage := "NextPointCloudFunc not implemented"
		cam := &inject.Camera{}
		cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
			return nil, errors.New(expectedErrorMessage)
		}
		deps := make(resource.Dependencies)
		deps[camera.Named(testLidarName)] = cam
		sensors := []string{testLidarName}
		sensorIndex := 0

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, testLidar, test.ShouldNotBeNil)
		test.That(t, testLidar.Name, test.ShouldEqual, testLidarName)

		pc, err := testLidar.GetData(context.Background())
		test.That(t, pc, test.ShouldBeNil)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldEqual, expectedErrorMessage)
	})

	t.Run("Successful GetData call", func(t *testing.T) {
		cam := &inject.Camera{}
		cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
			pc1 := pointcloud.New()
			err := pc1.Set(pointcloud.NewVector(1, 0, 0), pointcloud.NewColoredData(color.NRGBA{255, 0, 0, 255}))
			test.That(t, err, test.ShouldBeNil)
			return pc1, nil
		}
		deps := make(resource.Dependencies)
		deps[camera.Named(testLidarName)] = cam
		sensors := []string{testLidarName}
		sensorIndex := 0

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, testLidar, test.ShouldNotBeNil)
		test.That(t, testLidar.Name, test.ShouldEqual, testLidarName)

		pc, err := testLidar.GetData(context.Background())
		test.That(t, pc, test.ShouldNotBeNil)
		test.That(t, pc.Size(), test.ShouldEqual, 1)
		test.That(t, err, test.ShouldBeNil)

		data, got := pc.At(1, 0, 0)
		test.That(t, got, test.ShouldBeTrue)
		test.That(t, data.Color(), test.ShouldResemble, &color.NRGBA{255, 0, 0, 255})
	})
}

func TestTimedSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	sensors := []string{"invalid_sensor"}
	invalidLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
	test.That(t, err, test.ShouldBeNil)

	sensors = []string{"invalid_replay_sensor"}
	invalidReplayLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
	test.That(t, err, test.ShouldBeNil)

	sensors = []string{"good_lidar"}
	goodLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
	test.That(t, err, test.ShouldBeNil)

	sensors = []string{"replay_sensor"}
	goodReplayLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("when the lidar returns an error, returns that error", func(t *testing.T) {
		tsr, err := invalidLidar.TimedSensorReading(ctx)
		msg := "invalid sensor"
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
		test.That(t, tsr, test.ShouldResemble, s.TimedSensorReadingResponse{})
	})

	t.Run("when the replay lidar succeeds but the timestamp is invalid, returns an error", func(t *testing.T) {
		tsr, err := invalidReplayLidar.TimedSensorReading(ctx)
		msg := "parsing time \"NOT A TIME\" as \"2006-01-02T15:04:05.999999999Z07:00\": cannot parse \"NOT A TIME\" as \"2006\""
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
		test.That(t, tsr, test.ShouldResemble, s.TimedSensorReadingResponse{})
	})

	t.Run("when a live lidar succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		beforeReading := time.Now().UTC()
		tsr, err := goodLidar.TimedSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.Reading, test.ShouldNotBeNil)
		test.That(t, tsr.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, tsr.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, tsr.Replay, test.ShouldBeFalse)
	})

	t.Run("when a replay lidar succeeds, returns the replay sensor time and the reading", func(t *testing.T) {
		tsr, err := goodReplayLidar.TimedSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.Reading, test.ShouldNotBeNil)
		test.That(t, tsr.ReadingTime, test.ShouldEqual, time.Date(2006, 1, 2, 15, 4, 5, 999900000, time.UTC))
		test.That(t, tsr.Replay, test.ShouldBeTrue)
	})
}
