// Package sensors_test implements tests for sensors
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

func TestNewOdometer(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("No movement sensor provided", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.NoMovementSensor
		_, err := s.NewOdometer(context.Background(), s.SetupDeps(lidar, odometer), string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Failed odometer creation with non-existing movement sensor", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.GibberishMovementSensor
		actualOdometer, err := s.NewOdometer(context.Background(), s.SetupDeps(lidar, odometer), string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting movement sensor \""+string(odometer)+"\" for slam service: \""+
				"rdk:component:movement_sensor/"+string(odometer)+"\" missing from dependencies"))
		test.That(t, actualOdometer, test.ShouldResemble, s.Odometer{})
	})

	t.Run("Failed odometer creation with sensor that does not support Position", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.OdometerWithInvalidProperties
		actualOdometer, err := s.NewOdometer(context.Background(), s.SetupDeps(lidar, odometer), string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring odometer movement sensor error: "+
				"'movement_sensor' must support both Position and Orientation"))
		test.That(t, actualOdometer, test.ShouldResemble, s.Odometer{})
	})

	t.Run("Successful odometer creation", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.GoodOdometer
		actualOdometer, err := s.NewOdometer(context.Background(), s.SetupDeps(lidar, odometer), string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, actualOdometer.Name(), test.ShouldEqual, string(odometer))

		tsr, err := actualOdometer.TimedOdometerSensorReading(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.Position, test.ShouldResemble, s.Position)
		test.That(t, tsr.Orientation, test.ShouldResemble, s.Orientation)
	})
}

func TestTimedOdometerSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	t.Run("when the odometer's functions return an error, TimedOdometerSensorReading wraps that error", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.OdometerWithErroringFunctions
		actualOdometer, err := s.NewOdometer(ctx, s.SetupDeps(lidar, odometer), string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		tsr, err := actualOdometer.TimedOdometerSensorReading(ctx)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, s.InvalidSensorTestErrMsg)
		test.That(t, tsr, test.ShouldResemble, s.TimedOdometerSensorReadingResponse{})
	})

	t.Run("when a live odometer succeeds, returns current time in UTC and the reading", func(t *testing.T) {
		lidar, odometer := s.GoodLidar, s.GoodOdometer
		actualOdometer, err := s.NewOdometer(ctx, s.SetupDeps(lidar, odometer), string(odometer), testDataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		beforeReading := time.Now().UTC()
		time.Sleep(time.Millisecond)

		tsr, err := actualOdometer.TimedOdometerSensorReading(ctx)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, tsr.Position, test.ShouldResemble, s.Position)
		test.That(t, tsr.Orientation, test.ShouldResemble, s.Orientation)
		test.That(t, tsr.ReadingTime.After(beforeReading), test.ShouldBeTrue)
		test.That(t, tsr.ReadingTime.Location(), test.ShouldEqual, time.UTC)
		test.That(t, tsr.Replay, test.ShouldBeFalse)
	})
}
