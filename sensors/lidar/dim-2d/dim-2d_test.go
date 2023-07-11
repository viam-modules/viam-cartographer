// Package dim2d_test implements tests for the 2D sub algorithm
package dim2d_test

import (
	"context"
	"os"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.viam.com/test"

	internaltesthelper "github.com/viamrobotics/viam-cartographer/internal/testhelper"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
	dim2d "github.com/viamrobotics/viam-cartographer/sensors/lidar/dim-2d"
	"github.com/viamrobotics/viam-cartographer/testhelper"
)

func TestNewLidar(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("No sensor provided", func(t *testing.T) {
		sensors := []string{}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(context.Background(), deps, sensors, logger)
		expectedLidar := lidar.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Failed lidar creation due to more than one sensor provided", func(t *testing.T) {
		sensors := []string{"lidar", "one-too-many"}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(context.Background(), deps, sensors, logger)
		expectedLidar := lidar.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: 'sensors' must contain only one lidar camera, but is 'sensors: [lidar, one-too-many]'"))
	})

	t.Run("Failed lidar creation with non-existing sensor", func(t *testing.T) {
		sensors := []string{"gibberish"}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(context.Background(), deps, sensors, logger)
		expectedLidar := lidar.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: error getting lidar camera "+
				"gibberish for slam service: \"rdk:component:camera/gibberish\" missing from dependencies"))
	})

	t.Run("Successful lidar creation", func(t *testing.T) {
		sensors := []string{"good_lidar"}
		ctx := context.Background()
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(ctx, deps, sensors, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, sensors[0])
		test.That(t, err, test.ShouldBeNil)

		pc, err := actualLidar.GetData(ctx)
		test.That(t, pc, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeNil)
	})
}

func TestGetAndSaveData(t *testing.T) {
	ctx := context.Background()
	logger := golog.NewTestLogger(t)
	dataDir, err := internaltesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Successful call to GetAndSaveData", func(t *testing.T) {
		sensors := []string{"good_lidar"}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(ctx, deps, sensors, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, sensors[0])
		test.That(t, err, test.ShouldBeNil)

		_, err = dim2d.GetAndSaveData(ctx, dataDir, actualLidar, logger)
		test.That(t, err, test.ShouldBeNil)

		files, err := os.ReadDir(dataDir + "/data/")
		test.That(t, len(files), test.ShouldEqual, 1)
		test.That(t, err, test.ShouldBeNil)
	})

	internaltesthelper.ClearDirectory(t, dataDir)
}

func TestValidateGetAndSaveData(t *testing.T) {
	ctx := context.Background()
	logger := golog.NewTestLogger(t)
	dataDir, err := internaltesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Successful call to ValidateGetAndSaveData", func(t *testing.T) {
		sensors := []string{"good_lidar"}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(ctx, deps, sensors, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, sensors[0])
		test.That(t, err, test.ShouldBeNil)

		err = dim2d.ValidateGetAndSaveData(ctx,
			dataDir,
			actualLidar,
			internaltesthelper.SensorValidationMaxTimeoutSecForTest,
			internaltesthelper.SensorValidationMaxTimeoutSecForTest,
			logger,
		)
		test.That(t, err, test.ShouldBeNil)

		files, err := os.ReadDir(dataDir + "/data/")
		test.That(t, len(files), test.ShouldEqual, 0)
		test.That(t, err, test.ShouldBeNil)
	})

	internaltesthelper.ClearDirectory(t, dataDir)
}

func TestGetTimedData(t *testing.T) {
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
		_, pc, err := dim2d.GetTimedData(ctx, invalidLidar)
		test.That(t, err, test.ShouldBeError, errors.New("invalid sensor"))
		test.That(t, pc, test.ShouldBeNil)
	})

	t.Run("when the replay lidar succeeds but the timestamp is invalid, returns an error", func(t *testing.T) {
		_, pc, err := dim2d.GetTimedData(ctx, invalidReplayLidar)
		msg := "parsing time \"NOT A TIME\" as \"2006-01-02T15:04:05.999999999Z07:00\": cannot parse \"NOT A TIME\" as \"2006\""
		test.That(t, err, test.ShouldBeError, errors.New(msg))
		test.That(t, pc, test.ShouldBeNil)
	})

	t.Run("when a live lidar succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		beforeReading := time.Now().UTC()
		pcTime, pc, err := dim2d.GetTimedData(ctx, goodLidar)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc, test.ShouldNotBeNil)
		test.That(t, pcTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, pcTime.Location(), test.ShouldEqual, time.UTC)
	})

	t.Run("when a replay lidar succeeds, returns the replay sensor time and the reading", func(t *testing.T) {
		pcTime, pc, err := dim2d.GetTimedData(ctx, goodReplayLidar)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc, test.ShouldNotBeNil)
		test.That(t, pcTime, test.ShouldEqual, time.Date(2006, 1, 2, 15, 4, 5, 999900000, time.UTC))
	})
}

func TestValidateGetData(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	sensors := []string{"good_lidar"}
	goodLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
	test.That(t, err, test.ShouldBeNil)

	sensors = []string{"invalid_sensor"}
	invalidLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
	test.That(t, err, test.ShouldBeNil)

	sensorValidationMaxTimeout := time.Duration(50) * time.Millisecond
	sensorValidationInterval := time.Duration(10) * time.Millisecond

	t.Run("returns nil if a lidar reading succeeds immediately", func(t *testing.T) {
		err := dim2d.ValidateGetData(ctx, goodLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns nil if a lidar reading succeeds within the timeout", func(t *testing.T) {
		sensors = []string{"warming_up_lidar"}
		warmingUpLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
		test.That(t, err, test.ShouldBeNil)

		err = dim2d.ValidateGetData(ctx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns error if no lidar reading succeeds within the timeout", func(t *testing.T) {
		err := dim2d.ValidateGetData(ctx, invalidLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeError, errors.New("ValidateGetData timeout: invalid sensor"))
	})

	t.Run("returns error if no lidar reading succeeds by the time the context is cancelled", func(t *testing.T) {
		cancelledCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()

		sensors = []string{"warming_up_lidar"}
		warmingUpLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
		test.That(t, err, test.ShouldBeNil)

		err = dim2d.ValidateGetData(cancelledCtx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeError, context.Canceled)
	})
}
