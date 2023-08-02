// Package s_test implements tests for sensors
package sensors_test

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/test"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

func TestValidateGetData(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	lidar := "good_lidar"
	goodLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar = "invalid_lidar"
	invalidLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	// https://viam.atlassian.net/browse/RSDK-4306
	// test not relevant until replay camera supports Properties
	// lidar = map[string]string{"name": "no_pcd_camera"}
	// _, err = s.NewLidar(ctx, s.SetupDeps(lidar), lidar, logger)
	// test.That(t, err, test.ShouldBeError, errors.New("configuring lidar camera error: 'camera' must support PCD"))

	sensorValidationMaxTimeout := time.Duration(50) * time.Millisecond
	sensorValidationInterval := time.Duration(10) * time.Millisecond

	t.Run("returns nil if a lidar reading succeeds immediately", func(t *testing.T) {
		err := s.ValidateGetData(ctx, goodLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns nil if a lidar reading succeeds within the timeout", func(t *testing.T) {
		lidar = "warming_up_lidar"
		warmingUpLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar), lidar, logger)
		test.That(t, err, test.ShouldBeNil)
		err = s.ValidateGetData(ctx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns error if no lidar reading succeeds within the timeout", func(t *testing.T) {
		err := s.ValidateGetData(ctx, invalidLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeError, errors.New("ValidateGetData timeout: NextPointCloud error: invalid sensor"))
	})

	t.Run("returns error if no lidar reading succeeds by the time the context is cancelled", func(t *testing.T) {
		cancelledCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()

		lidar = "warming_up_lidar"
		warmingUpLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar), lidar, logger)
		test.That(t, err, test.ShouldBeNil)

		err = s.ValidateGetData(cancelledCtx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeError, context.Canceled)
	})
}

func TestNewLidar(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("No sensor provided", func(t *testing.T) {
		lidar := ""
		deps := s.SetupDeps(lidar)
		_, err := s.NewLidar(context.Background(), deps, lidar, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				" for slam service: \"rdk:component:camera/\" missing from dependencies"))
	})

	t.Run("Failed lidar creation with non-existing sensor", func(t *testing.T) {
		lidar := "gibberish"
		deps := s.SetupDeps(lidar)
		actualLidar, err := s.NewLidar(context.Background(), deps, lidar, logger)
		expectedLidar := s.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				"gibberish for slam service: \"rdk:component:camera/gibberish\" missing from dependencies"))
	})

	t.Run("Successful lidar creation", func(t *testing.T) {
		lidar := "good_lidar"
		ctx := context.Background()
		deps := s.SetupDeps(lidar)
		actualLidar, err := s.NewLidar(ctx, deps, lidar, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, lidar)
		test.That(t, err, test.ShouldBeNil)

		tsr, err := actualLidar.TimedLidarSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(tsr.Reading), test.ShouldBeGreaterThan, 0)
	})
}

func TestTimedSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	lidar := "invalid_lidar"
	invalidLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar = "invalid_replay_lidar"
	invalidReplayLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar = "good_lidar"
	goodLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar = "replay_lidar"
	goodReplayLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("when the lidar returns an error, returns that error", func(t *testing.T) {
		tsr, err := invalidLidar.TimedLidarSensorReading(ctx)
		msg := "invalid sensor"
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
		test.That(t, tsr, test.ShouldResemble, s.TimedSensorReadingResponse{})
	})

	t.Run("when the replay lidar succeeds but the timestamp is invalid, returns an error", func(t *testing.T) {
		tsr, err := invalidReplayLidar.TimedLidarSensorReading(ctx)
		msg := "parsing time \"NOT A TIME\" as \"2006-01-02T15:04:05.999999999Z07:00\": cannot parse \"NOT A TIME\" as \"2006\""
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
		test.That(t, tsr, test.ShouldResemble, s.TimedSensorReadingResponse{})
	})

	t.Run("when a live lidar succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		beforeReading := time.Now().UTC()
		tsr, err := goodLidar.TimedLidarSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.Reading, test.ShouldNotBeNil)
		result := "VERSION .7\nFIELDS x y z\n" +
			"SIZE 4 4 4\nTYPE F F F\n" +
			"COUNT 1 1 1\nWIDTH 0\nHEIGHT 1\n" +
			"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\n" +
			"DATA binary\n"
		test.That(t, tsr.Reading, test.ShouldResemble, []byte(result))
		test.That(t, tsr.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, tsr.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, tsr.Replay, test.ShouldBeFalse)
	})

	t.Run("when a replay lidar succeeds, returns the replay sensor time and the reading", func(t *testing.T) {
		tsr, err := goodReplayLidar.TimedLidarSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.Reading, test.ShouldNotBeNil)
		test.That(t, tsr.ReadingTime, test.ShouldEqual, time.Date(2006, 1, 2, 15, 4, 5, 999900000, time.UTC))
		test.That(t, tsr.Replay, test.ShouldBeTrue)
	})
}
