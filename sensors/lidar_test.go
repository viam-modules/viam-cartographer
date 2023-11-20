package sensors_test

import (
	"context"
	"testing"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/logging"
	"go.viam.com/test"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

func TestNewLidar(t *testing.T) {
	logger := logging.NewTestLogger(t)

	t.Run("No lidar provided", func(t *testing.T) {
		lidar, imu := s.NoLidar, s.NoIMU
		_, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				" for slam service: \"rdk:component:camera/\" missing from dependencies"))
	})

	t.Run("Failed lidar creation with non-existing sensor", func(t *testing.T) {
		lidar, imu := s.GibberishLidar, s.NoIMU
		actualLidar, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				"gibberish_lidar for slam service: \"rdk:component:camera/gibberish_lidar\" missing from dependencies"))
		test.That(t, actualLidar, test.ShouldResemble, s.Lidar{})
	})

	t.Run("Successful lidar creation", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.NoIMU
		actualLidar, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
		test.That(t, actualLidar.Name(), test.ShouldEqual, string(lidar))
		test.That(t, err, test.ShouldBeNil)

		tsr, err := actualLidar.TimedLidarSensorReading(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(tsr.Reading), test.ShouldBeGreaterThan, 0)
	})
}

func TestValidateGetLidarData(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	lidar, imu := s.GoodLidar, s.NoIMU
	goodLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar, imu = s.LidarWithInvalidProperties, s.NoIMU
	_, err = s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeError, errors.New("configuring lidar camera error: 'camera' must support PCD"))

	sensorValidationMaxTimeout := time.Duration(50) * time.Millisecond
	sensorValidationInterval := time.Duration(10) * time.Millisecond

	t.Run("returns nil if a lidar reading succeeds immediately", func(t *testing.T) {
		err := s.ValidateGetLidarData(ctx, goodLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns nil if a lidar reading succeeds within the timeout", func(t *testing.T) {
		lidar, imu = s.WarmingUpLidar, s.NoIMU
		warmingUpLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
		err = s.ValidateGetLidarData(ctx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns error if no lidar reading succeeds by the time the context is cancelled", func(t *testing.T) {
		cancelledCtx, cancelFunc := context.WithCancel(ctx)
		cancelFunc()

		lidar, imu = s.WarmingUpLidar, s.NoIMU
		warmingUpLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		err = s.ValidateGetLidarData(cancelledCtx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeError, context.Canceled)
	})
}

func TestTimedLidarSensorReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	lidar, imu := s.LidarWithErroringFunctions, s.NoIMU
	lidarWithErroringFunctions, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar, imu = s.InvalidReplayLidar, s.NoIMU
	invalidReplayLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar, imu = s.GoodLidar, s.NoIMU
	goodLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar, imu = s.ReplayLidar, s.NoIMU
	goodReplayLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("when the lidar returns an error, returns that error", func(t *testing.T) {
		tsr, err := lidarWithErroringFunctions.TimedLidarSensorReading(ctx)
		msg := "invalid sensor"
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
		test.That(t, tsr, test.ShouldResemble, s.TimedLidarSensorReadingResponse{})
	})

	t.Run("when the replay lidar succeeds but the timestamp is invalid, returns an error", func(t *testing.T) {
		tsr, err := invalidReplayLidar.TimedLidarSensorReading(ctx)
		msg := "parsing time \"NOT A TIME\" as \"2006-01-02T15:04:05.999999999Z07:00\": cannot parse \"NOT A TIME\" as \"2006\""
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
		test.That(t, tsr, test.ShouldResemble, s.TimedLidarSensorReadingResponse{})
	})

	t.Run("when a live lidar succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		beforeReading := time.Now().UTC()
		time.Sleep(10 * time.Millisecond)

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

		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTimestamp)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.ReadingTime.Equal(readingTime), test.ShouldBeTrue)

		test.That(t, tsr.Replay, test.ShouldBeTrue)
	})
}
