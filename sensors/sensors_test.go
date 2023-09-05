// Package s_test implements tests for sensors
package sensors_test

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/test"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

func TestValidateGetLidarData(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	lidar := "good_lidar"
	goodLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, ""), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar = "lidar_with_invalid_properties"
	_, err = s.NewLidar(ctx, s.SetupDeps(lidar, ""), lidar, logger)
	test.That(t, err, test.ShouldBeError, errors.New("configuring lidar camera error: 'camera' must support PCD"))

	sensorValidationMaxTimeout := time.Duration(50) * time.Millisecond
	sensorValidationInterval := time.Duration(10) * time.Millisecond

	t.Run("returns nil if a lidar reading succeeds immediately", func(t *testing.T) {
		err := s.ValidateGetLidarData(ctx, goodLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns nil if a lidar reading succeeds within the timeout", func(t *testing.T) {
		lidar = "warming_up_lidar"
		warmingUpLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, ""), lidar, logger)
		test.That(t, err, test.ShouldBeNil)
		err = s.ValidateGetLidarData(ctx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns error if no lidar reading succeeds by the time the context is cancelled", func(t *testing.T) {
		cancelledCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()

		lidar = "warming_up_lidar"
		warmingUpLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, ""), lidar, logger)
		test.That(t, err, test.ShouldBeNil)

		err = s.ValidateGetLidarData(cancelledCtx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeError, context.Canceled)
	})
}

func TestNewLidar(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("No lidar provided", func(t *testing.T) {
		lidar := ""
		deps := s.SetupDeps(lidar, "")
		_, err := s.NewLidar(context.Background(), deps, lidar, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				" for slam service: \"rdk:component:camera/\" missing from dependencies"))
	})

	t.Run("Failed lidar creation with non-existing sensor", func(t *testing.T) {
		lidar := "gibberish"
		deps := s.SetupDeps(lidar, "")
		actualLidar, err := s.NewLidar(context.Background(), deps, lidar, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				"gibberish for slam service: \"rdk:component:camera/gibberish\" missing from dependencies"))
		expectedLidar := s.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
	})

	t.Run("Successful lidar creation", func(t *testing.T) {
		lidar := "good_lidar"
		ctx := context.Background()
		deps := s.SetupDeps(lidar, "")
		actualLidar, err := s.NewLidar(ctx, deps, lidar, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, lidar)
		test.That(t, err, test.ShouldBeNil)

		tsr, err := actualLidar.TimedLidarSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(tsr.Reading), test.ShouldBeGreaterThan, 0)
	})
}

func TestNewIMU(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("No IMU provided", func(t *testing.T) {
		imu := ""
		lidar := "good_lidar"
		deps := s.SetupDeps(lidar, imu)
		_, err := s.NewIMU(context.Background(), deps, imu, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Failed IMU creation with non-existing sensor", func(t *testing.T) {
		lidar := "good_lidar"
		imu := "gibberish"
		deps := s.SetupDeps(lidar, imu)
		actualIMU, err := s.NewIMU(context.Background(), deps, imu, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting IMU movement sensor "+
				"gibberish for slam service: \"rdk:component:movement_sensor/gibberish\" missing from dependencies"))
		expectedIMU := s.IMU{}
		test.That(t, actualIMU, test.ShouldResemble, expectedIMU)
	})

	t.Run("Failed IMU creation with sensor that does not support AngularVelocity", func(t *testing.T) {
		lidar := "good_lidar"
		imu := "imu_with_invalid_properties"
		deps := s.SetupDeps(lidar, imu)
		actualIMU, err := s.NewIMU(context.Background(), deps, imu, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring IMU movement sensor error: "+
				"'movement_sensor' must support both LinearAcceleration and AngularVelocity"))
		expectedIMU := s.IMU{}
		test.That(t, actualIMU, test.ShouldResemble, expectedIMU)
	})

	t.Run("Successful IMU creation", func(t *testing.T) {
		lidar := "good_lidar"
		imu := "good_imu"
		ctx := context.Background()
		deps := s.SetupDeps(lidar, imu)
		actualIMU, err := s.NewIMU(ctx, deps, imu, logger)
		test.That(t, actualIMU.Name, test.ShouldEqual, imu)
		test.That(t, err, test.ShouldBeNil)

		tsr, err := actualIMU.TimedIMUSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.LinearAcceleration, test.ShouldResemble, s.LinAcc)
		test.That(t, tsr.AngularVelocity, test.ShouldResemble,
			spatialmath.AngularVelocity{
				X: rdkutils.DegToRad(s.AngVel.X),
				Y: rdkutils.DegToRad(s.AngVel.Y),
				Z: rdkutils.DegToRad(s.AngVel.Z),
			})
	})
}

func TestTimedLidarSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	lidar := "lidar_with_erroring_functions"
	lidarWithErroringFunctions, err := s.NewLidar(ctx, s.SetupDeps(lidar, ""), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar = "invalid_replay_lidar"
	invalidReplayLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, ""), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar = "good_lidar"
	goodLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, ""), lidar, logger)
	test.That(t, err, test.ShouldBeNil)

	lidar = "replay_lidar"
	goodReplayLidar, err := s.NewLidar(ctx, s.SetupDeps(lidar, ""), lidar, logger)
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
		test.That(t, tsr.ReadingTime.Equal(s.TestTime), test.ShouldBeTrue)
		test.That(t, tsr.Replay, test.ShouldBeTrue)
	})
}

func TestTimedIMUSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	lidar := "good_lidar"
	imu := "imu_with_erroring_functions"
	imuWithErroringFunctions, err := s.NewIMU(ctx, s.SetupDeps(lidar, imu), imu, logger)
	test.That(t, err, test.ShouldBeNil)

	imu = "good_imu"
	goodIMU, err := s.NewIMU(ctx, s.SetupDeps(lidar, imu), imu, logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("when the IMU returns an error, returns that error", func(t *testing.T) {
		tsr, err := imuWithErroringFunctions.TimedIMUSensorReading(ctx)
		msg := "invalid sensor"
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, msg)
		test.That(t, tsr, test.ShouldResemble, s.TimedIMUSensorReadingResponse{})
	})

	t.Run("when a live IMU succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		beforeReading := time.Now().UTC()
		time.Sleep(time.Millisecond)

		tsr, err := goodIMU.TimedIMUSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.LinearAcceleration, test.ShouldResemble, s.LinAcc)
		test.That(t, tsr.AngularVelocity, test.ShouldResemble,
			spatialmath.AngularVelocity{
				X: rdkutils.DegToRad(s.AngVel.X),
				Y: rdkutils.DegToRad(s.AngVel.Y),
				Z: rdkutils.DegToRad(s.AngVel.Z),
			})
		test.That(t, tsr.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, tsr.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, tsr.Replay, test.ShouldBeFalse)
	})
}
