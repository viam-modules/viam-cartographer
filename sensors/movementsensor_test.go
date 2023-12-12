// Package sensors_test implements tests for sensors
package sensors_test

import (
	"context"
	"errors"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/test"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

const (
	testDataFrequencyHz = 1
)

func TestNewMovementSensor(t *testing.T) {
	logger := logging.NewTestLogger(t)

	t.Run("No movement sensor provided", func(t *testing.T) {
		lidar, movementSensor := s.GoodLidar, s.NoMovementSensor
		deps := s.SetupDeps(lidar, movementSensor)
		actualMs, err := s.NewMovementSensor(context.Background(), deps, string(movementSensor), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualMs, test.ShouldResemble, &s.MovementSensor{})
	})

	t.Run("Failed movement sensor creation with non-existing movement sensor", func(t *testing.T) {
		lidar, movementSensor := s.GoodLidar, s.GibberishMovementSensor
		deps := s.SetupDeps(lidar, movementSensor)
		actualMs, err := s.NewMovementSensor(context.Background(), deps, string(movementSensor), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting movement sensor \""+string(movementSensor)+"\" for slam service: "+
				"Resource missing from dependencies. Resource: rdk:component:movement_sensor/"+string(movementSensor)))
		test.That(t, actualMs, test.ShouldResemble, &s.MovementSensor{})
	})

	t.Run("Failed movement creation with sensor that does not support IMU nor odometer", func(t *testing.T) {
		lidar, movementSensor := s.GoodLidar, s.MovementSensorWithInvalidProperties
		deps := s.SetupDeps(lidar, movementSensor)
		actualMs, err := s.NewMovementSensor(context.Background(), deps, string(movementSensor), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError, s.ErrMovementSensorNeitherIMUNorOdometer)
		test.That(t, actualMs, test.ShouldResemble, &s.MovementSensor{})
	})

	t.Run("Successful movement sensor creation that supports an IMU", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.GoodIMU
		deps := s.SetupDeps(lidar, imu)
		actualMs, err := s.NewMovementSensor(context.Background(), deps, string(imu), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualMs.Name(), test.ShouldEqual, string(imu))

		actualReading, err := actualMs.TimedMovementSensorReading(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualReading.TimedIMUResponse.LinearAcceleration, test.ShouldResemble, s.TestLinAcc)
		test.That(t, actualReading.TimedIMUResponse.AngularVelocity, test.ShouldResemble,
			spatialmath.AngularVelocity{
				X: rdkutils.DegToRad(s.TestAngVel.X),
				Y: rdkutils.DegToRad(s.TestAngVel.Y),
				Z: rdkutils.DegToRad(s.TestAngVel.Z),
			})
		test.That(t, actualReading.TimedOdometerResponse, test.ShouldBeNil)
	})

	t.Run("Successful movement sensor creation that supports an odometer", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.GoodOdometer
		deps := s.SetupDeps(lidar, odometer)
		actualMs, err := s.NewMovementSensor(context.Background(), deps, string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualMs.Name(), test.ShouldEqual, string(odometer))

		actualReading, err := actualMs.TimedMovementSensorReading(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualReading.TimedOdometerResponse.Position, test.ShouldResemble, s.TestPosition)
		test.That(t, actualReading.TimedOdometerResponse.Orientation, test.ShouldResemble, s.TestOrientation)
		test.That(t, actualReading.TimedIMUResponse, test.ShouldBeNil)
	})
}

func TestProperties(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	t.Run("only IMU supported", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.GoodIMU
		deps := s.SetupDeps(lidar, imu)
		actualIMU, err := s.NewMovementSensor(ctx, deps, string(imu), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
		properties := actualIMU.Properties()
		test.That(t, properties.IMUSupported, test.ShouldBeTrue)
		test.That(t, properties.OdometerSupported, test.ShouldBeFalse)
	})

	t.Run("only odometer supported", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.GoodOdometer
		deps := s.SetupDeps(lidar, odometer)
		actualOdometer, err := s.NewMovementSensor(ctx, deps, string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
		properties := actualOdometer.Properties()
		test.That(t, properties.IMUSupported, test.ShouldBeFalse)
		test.That(t, properties.OdometerSupported, test.ShouldBeTrue)
	})

	t.Run("both IMU and odometer supported", func(t *testing.T) {
		lidar, movementSensor := s.GoodLidar, s.GoodMovementSensorBothIMUAndOdometer
		deps := s.SetupDeps(lidar, movementSensor)
		actualMovementSensor, err := s.NewMovementSensor(ctx, deps, string(movementSensor), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
		properties := actualMovementSensor.Properties()
		test.That(t, properties.IMUSupported, test.ShouldBeTrue)
		test.That(t, properties.OdometerSupported, test.ShouldBeTrue)
	})
}

func TestTimedMovementSensorReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	t.Run("when the movement sensor's IMU functions return an error, timedIMUReading wraps that error", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.IMUWithErroringFunctions
		deps := s.SetupDeps(lidar, imu)
		actualIMU, err := s.NewMovementSensor(ctx, deps, string(imu), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		actualReading, err := actualIMU.TimedMovementSensorReading(ctx)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, s.InvalidSensorTestErrMsg)
		test.That(t, actualReading, test.ShouldResemble, s.TimedMovementSensorReadingResponse{})
	})

	t.Run("when the movement sensor's odometer functions return an error, timedOdometerReading wraps that error", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.OdometerWithErroringFunctions
		deps := s.SetupDeps(lidar, odometer)
		actualOdometer, err := s.NewMovementSensor(ctx, deps, string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		actualReading, err := actualOdometer.TimedMovementSensorReading(ctx)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, s.InvalidSensorTestErrMsg)
		test.That(t, actualReading, test.ShouldResemble, s.TimedMovementSensorReadingResponse{})
	})

	t.Run("when a live IMU succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		lidar, imu := s.GoodLidar, s.GoodIMU
		deps := s.SetupDeps(lidar, imu)
		actualIMU, err := s.NewMovementSensor(ctx, deps, string(imu), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		beforeReading := time.Now().UTC()
		time.Sleep(time.Millisecond)

		actualReading, err := actualIMU.TimedMovementSensorReading(ctx)

		time.Sleep(time.Millisecond)
		afterReading := time.Now().UTC()

		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualReading.TimedIMUResponse.LinearAcceleration, test.ShouldResemble, s.TestLinAcc)
		test.That(t, actualReading.TimedIMUResponse.AngularVelocity, test.ShouldResemble,
			spatialmath.AngularVelocity{
				X: rdkutils.DegToRad(s.TestAngVel.X),
				Y: rdkutils.DegToRad(s.TestAngVel.Y),
				Z: rdkutils.DegToRad(s.TestAngVel.Z),
			})
		test.That(t, actualReading.TimedIMUResponse.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, actualReading.TimedIMUResponse.ReadingTime.Before(afterReading), test.ShouldBeTrue)
		test.That(t, actualReading.TimedIMUResponse.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, actualReading.TimedOdometerResponse, test.ShouldBeNil)
		test.That(t, actualReading.TestIsReplaySensor, test.ShouldBeFalse)
	})

	t.Run("when a live odometer succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.GoodOdometer
		deps := s.SetupDeps(lidar, odometer)
		actualOdometer, err := s.NewMovementSensor(ctx, deps, string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		beforeReading := time.Now().UTC()
		time.Sleep(time.Millisecond)

		actualReading, err := actualOdometer.TimedMovementSensorReading(ctx)

		time.Sleep(time.Millisecond)
		afterReading := time.Now().UTC()

		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualReading.TimedOdometerResponse.Position, test.ShouldResemble, s.TestPosition)
		test.That(t, actualReading.TimedOdometerResponse.Orientation, test.ShouldResemble, s.TestOrientation)
		test.That(t, actualReading.TimedOdometerResponse.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, actualReading.TimedOdometerResponse.ReadingTime.Before(afterReading), test.ShouldBeTrue)
		test.That(t, actualReading.TimedOdometerResponse.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, actualReading.TimedIMUResponse, test.ShouldBeNil)
		test.That(t, actualReading.TestIsReplaySensor, test.ShouldBeFalse)
	})

	t.Run("when a movemement sensor that supports both an odometer and an IMU succeeds,"+
		" returns current time in UTC and the reading", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.GoodMovementSensorBothIMUAndOdometer
		deps := s.SetupDeps(lidar, odometer)
		actualOdometer, err := s.NewMovementSensor(ctx, deps, string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		beforeReading := time.Now().UTC()
		time.Sleep(time.Millisecond)

		actualReading, err := actualOdometer.TimedMovementSensorReading(ctx)

		time.Sleep(time.Millisecond)
		afterReading := time.Now().UTC()

		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualReading.TimedOdometerResponse.Position, test.ShouldResemble, s.TestPosition)
		test.That(t, actualReading.TimedOdometerResponse.Orientation, test.ShouldResemble, s.TestOrientation)
		test.That(t, actualReading.TimedOdometerResponse.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, actualReading.TimedOdometerResponse.ReadingTime.Before(afterReading), test.ShouldBeTrue)
		test.That(t, actualReading.TimedOdometerResponse.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, actualReading.TimedIMUResponse.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, actualReading.TimedIMUResponse.ReadingTime.Before(afterReading), test.ShouldBeTrue)
		test.That(t, actualReading.TimedIMUResponse.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, actualReading.TestIsReplaySensor, test.ShouldBeFalse)
	})
}
