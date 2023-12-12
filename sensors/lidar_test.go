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
		lidar, imu := s.NoLidar, s.NoMovementSensor
		actualLidar, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				" for slam service: Resource missing from dependencies. Resource: rdk:component:camera/"))
		test.That(t, actualLidar, test.ShouldResemble, s.Lidar{})
	})

	t.Run("Failed lidar creation with non-existing sensor", func(t *testing.T) {
		lidar, imu := s.GibberishLidar, s.NoMovementSensor
		actualLidar, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				"gibberish_lidar for slam service: Resource missing from dependencies. Resource: rdk:component:camera/gibberish_lidar"))
		test.That(t, actualLidar, test.ShouldResemble, s.Lidar{})
	})

	t.Run("Successful lidar creation", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.NoMovementSensor
		actualLidar, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
		test.That(t, actualLidar.Name(), test.ShouldEqual, string(lidar))
		test.That(t, err, test.ShouldBeNil)

		tsr, err := actualLidar.TimedLidarReading(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(tsr.Reading), test.ShouldBeGreaterThan, 0)
	})
}

func TestTimedLidarSensorReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	lidar, imu := s.LidarWithErroringFunctions, s.NoMovementSensor
	lidarWithErroringFunctions, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar, imu = s.InvalidReplayLidar, s.NoMovementSensor
	invalidReplayLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar, imu = s.GoodLidar, s.NoMovementSensor
	goodLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar, imu = s.ReplayLidar, s.NoMovementSensor
	goodReplayLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, imu), string(lidar), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("when the lidar returns an error, returns that error", func(t *testing.T) {
		tsr, err := lidarWithErroringFunctions.TimedLidarReading(ctx)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, s.InvalidSensorTestErrMsg)
		test.That(t, tsr, test.ShouldResemble, s.TimedLidarReadingResponse{})
	})

	t.Run("when the replay lidar succeeds but the timestamp is invalid, returns an error", func(t *testing.T) {
		tsr, err := invalidReplayLidar.TimedLidarReading(ctx)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring,
			"parsing time \"NOT A TIME\" as \"2006-01-02T15:04:05.999999999Z07:00\": cannot parse \"NOT A TIME\" as \"2006\"")
		test.That(t, tsr, test.ShouldResemble, s.TimedLidarReadingResponse{})
	})

	t.Run("when a live lidar succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		beforeReading := time.Now().UTC()
		time.Sleep(10 * time.Millisecond)

		tsr, err := goodLidar.TimedLidarReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.Reading, test.ShouldNotBeNil)
		expectedResult := "VERSION .7\nFIELDS x y z\n" +
			"SIZE 4 4 4\nTYPE F F F\n" +
			"COUNT 1 1 1\nWIDTH 0\nHEIGHT 1\n" +
			"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\n" +
			"DATA binary\n"
		test.That(t, tsr.Reading, test.ShouldResemble, []byte(expectedResult))
		test.That(t, tsr.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, tsr.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, tsr.TestIsReplaySensor, test.ShouldBeFalse)
	})

	t.Run("when a replay lidar succeeds, returns the replay sensor time and the reading", func(t *testing.T) {
		tsr, err := goodReplayLidar.TimedLidarReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.Reading, test.ShouldNotBeNil)

		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTimestamp)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.ReadingTime.Equal(readingTime), test.ShouldBeTrue)

		test.That(t, tsr.TestIsReplaySensor, test.ShouldBeTrue)
	})
}
