// Package sensors_test implements tests for sensors
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

const (
	testDataFrequencyHz = 1
)

func TestNewIMU(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("No IMU provided", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.NoIMU
		deps := s.SetupDeps(lidar, imu)
		_, err := s.NewIMU(context.Background(), deps, string(imu), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Failed IMU creation with non-existing sensor", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.GibberishIMU
		expectedIMU := s.IMU{}
		actualIMU, err := s.NewIMU(context.Background(), s.SetupDeps(lidar, imu), string(imu), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting movement sensor "+
				"\"gibberish_imu\" for slam service: \"rdk:component:movement_sensor/gibberish_imu\" missing from dependencies"))
		test.That(t, actualIMU, test.ShouldResemble, expectedIMU)
	})

	// TODO[kat]: Rewrite/change this with the MovementSensorNotIMUNotOdometer movement sensor
	t.Run("Failed IMU creation with sensor that does not support AngularVelocity", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.IMUWithInvalidProperties
		actualIMU, err := s.NewIMU(context.Background(), s.SetupDeps(lidar, imu), string(imu), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring IMU movement sensor error: "+
				"'movement_sensor' must support both LinearAcceleration and AngularVelocity"))
		expectedIMU := s.IMU{}
		test.That(t, actualIMU, test.ShouldResemble, expectedIMU)
	})

	t.Run("Successful IMU creation", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.GoodIMU
		actualIMU, err := s.NewIMU(context.Background(), s.SetupDeps(lidar, imu), string(imu), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualIMU.Name(), test.ShouldEqual, string(imu))

		tsr, err := actualIMU.TimedIMUSensorReading(context.Background())
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

func TestTimedIMUSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	lidar, imu := s.GoodLidar, s.IMUWithErroringFunctions
	imuWithErroringFunctions, err := s.NewIMU(ctx, s.SetupDeps(lidar, imu), string(imu), testDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	imu = s.GoodIMU
	goodIMU, err := s.NewIMU(ctx, s.SetupDeps(lidar, imu), string(imu), testDataFrequencyHz, logger)
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
