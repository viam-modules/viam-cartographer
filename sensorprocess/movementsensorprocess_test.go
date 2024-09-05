package sensorprocess

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viam-modules/viam-cartographer/cartofacade"
	s "github.com/viam-modules/viam-cartographer/sensors"
	"github.com/viam-modules/viam-cartographer/sensors/inject"
)

func TestStartMovementSensor(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}

	t.Run("exits loop when the context was cancelled", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		lidar, imu := s.NoLidar, s.FinishedReplayIMU
		replaySensor, err := s.NewMovementSensor(context.Background(), s.SetupDeps(lidar, imu), string(imu), 20, logger)
		test.That(t, err, test.ShouldBeNil)

		config.MovementSensor = replaySensor

		cancelFunc()

		config.StartMovementSensor(cancelCtx)
	})
}

func TestAddMovementSensorReadingInOnline(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	injectMovementSensor := inject.TimedMovementSensor{}
	injectMovementSensor.NameFunc = func() string { return "good_movement_sensor" }
	injectMovementSensor.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       true,
		MovementSensor: &injectMovementSensor,
		Timeout:        10 * time.Second,
	}

	t.Run("returns error when LinearAcceleration or AngularVelocity return an error, doesn't try to add IMU data", func(t *testing.T) {
		invalidAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, cf, config, s.IMUWithErroringFunctions)
	})

	t.Run("returns error when Position or Orientation return an error, doesn't try to add odometer data", func(t *testing.T) {
		invalidAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, cf, config, s.OdometerWithErroringFunctions)
	})

	t.Run("returns error when LinearAcceleration, AngularVelocity, Position, and Orientation return an error, "+
		"doesn't try to add movement sensor data", func(t *testing.T) {
		invalidAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, cf, config,
			s.MovementSensorBothIMUAndOdometerWithErroringFunctions)
	})

	t.Run("returns error when IMU replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		invalidAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, cf, config, s.InvalidReplayIMU)
	})

	t.Run("returns error when odometer replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		invalidAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, cf, config, s.InvalidReplayOdometer)
	})

	t.Run("returns error when replay movement sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		invalidAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, cf, config, s.InvalidReplayMovementSensorBothIMUAndOdometer)
	})

	t.Run("online replay IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		imuCalls, odometerCalls := validAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, config, cf, s.ReplayIMU)
		test.That(t, len(odometerCalls), test.ShouldBeZeroValue)
		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTimestamp)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, imuCalls[0].currentReading.ReadingTime.Equal(readingTime), test.ShouldBeTrue)
	})

	t.Run("online replay odometer adds sensor reading once and ignores errors", func(t *testing.T) {
		imuCalls, odometerCalls := validAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, config, cf, s.ReplayOdometer)
		test.That(t, len(imuCalls), test.ShouldBeZeroValue)
		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTimestamp)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, odometerCalls[0].currentReading.ReadingTime.Equal(readingTime), test.ShouldBeTrue)
	})

	t.Run("online replay movement sensor adds sensor reading once and ignores errors", func(t *testing.T) {
		imuCalls, odometerCalls := validAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, config, cf,
			s.ReplayMovementSensorBothIMUAndOdometer)
		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTimestamp)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, imuCalls[0].currentReading.ReadingTime.Equal(readingTime), test.ShouldBeTrue)
		test.That(t, odometerCalls[0].currentReading.ReadingTime.Equal(readingTime), test.ShouldBeTrue)
	})

	t.Run("online IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		imuCalls, odometerCalls := validAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, config, cf,
			s.GoodIMU)
		test.That(t, len(odometerCalls), test.ShouldBeZeroValue)
		test.That(t, imuCalls[0].currentReading.ReadingTime.Before(imuCalls[1].currentReading.ReadingTime), test.ShouldBeTrue)
		test.That(t, imuCalls[1].currentReading.ReadingTime.Before(imuCalls[2].currentReading.ReadingTime), test.ShouldBeTrue)
	})

	t.Run("online odometer adds sensor reading once and ignores errors", func(t *testing.T) {
		imuCalls, odometerCalls := validAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, config, cf,
			s.GoodOdometer)
		test.That(t, len(imuCalls), test.ShouldBeZeroValue)
		test.That(t, odometerCalls[0].currentReading.ReadingTime.Before(odometerCalls[1].currentReading.ReadingTime), test.ShouldBeTrue)
		test.That(t, odometerCalls[1].currentReading.ReadingTime.Before(odometerCalls[2].currentReading.ReadingTime), test.ShouldBeTrue)
	})

	t.Run("online movement sensor adds sensor reading once and ignores errors", func(t *testing.T) {
		imuCalls, odometerCalls := validAddMovementSensorReadingInOnlineTestHelper(context.Background(), t, config, cf,
			s.GoodMovementSensorBothIMUAndOdometer)
		test.That(t, imuCalls[0].currentReading.ReadingTime.Before(imuCalls[1].currentReading.ReadingTime), test.ShouldBeTrue)
		test.That(t, imuCalls[1].currentReading.ReadingTime.Before(imuCalls[2].currentReading.ReadingTime), test.ShouldBeTrue)
		test.That(t, odometerCalls[0].currentReading.ReadingTime.Before(odometerCalls[1].currentReading.ReadingTime), test.ShouldBeTrue)
		test.That(t, odometerCalls[1].currentReading.ReadingTime.Before(odometerCalls[2].currentReading.ReadingTime), test.ShouldBeTrue)
	})
}

func TestTryAddMovementSensorReadingUntilSuccess(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	cf := cartofacade.Mock{}

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    false,
		Timeout:     10 * time.Second,
	}

	t.Run("replay IMU attempts to add sensor data until success", func(t *testing.T) {
		validAddMovementSensorReadingUntilSuccessTestHelper(ctx, t, config, cf, s.ReplayIMU)
	})

	t.Run("replay odometer attempts to add sensor data until success", func(t *testing.T) {
		validAddMovementSensorReadingUntilSuccessTestHelper(ctx, t, config, cf, s.ReplayOdometer)
	})

	t.Run("replay movement sensor attempts to add sensor data until success", func(t *testing.T) {
		validAddMovementSensorReadingUntilSuccessTestHelper(ctx, t, config, cf, s.ReplayMovementSensorBothIMUAndOdometer)
	})
}

func TestTryAddMovementSensorReadingOnce(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	movementSensorName := "good_movement_sensor"
	injectMovementSensor := inject.TimedMovementSensor{}
	injectMovementSensor.NameFunc = func() string { return movementSensorName }
	injectMovementSensor.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:          &injectLidar,
		MovementSensor: &injectMovementSensor,
		Timeout:        10 * time.Second,
	}

	t.Run("imu only supported", func(t *testing.T) {
		injectMovementSensor.PropertiesFunc = func() s.MovementSensorProperties {
			return s.MovementSensorProperties{
				IMUSupported:      true,
				OdometerSupported: false,
			}
		}
		config.MovementSensor = &injectMovementSensor
		tryAddMovementSensorReadingOnceTestHelper(t, config, cf)
	})

	t.Run("odometer only supported", func(t *testing.T) {
		injectMovementSensor.PropertiesFunc = func() s.MovementSensorProperties {
			return s.MovementSensorProperties{
				IMUSupported:      true,
				OdometerSupported: false,
			}
		}
		config.MovementSensor = &injectMovementSensor
		tryAddMovementSensorReadingOnceTestHelper(t, config, cf)
	})

	t.Run("both imu and odometer supported", func(t *testing.T) {
		injectMovementSensor.PropertiesFunc = func() s.MovementSensorProperties {
			return s.MovementSensorProperties{
				IMUSupported:      true,
				OdometerSupported: false,
			}
		}
		config.MovementSensor = &injectMovementSensor
		tryAddMovementSensorReadingOnceTestHelper(t, config, cf)
	})
}

func TestTryAddIMUReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	imuReading := s.TimedIMUReadingResponse{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 1, Y: 1, Z: 1},
		ReadingTime:        time.Now().UTC(),
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	injectImu := inject.TimedMovementSensor{}
	injectImu.NameFunc = func() string { return "good_imu" }
	injectImu.DataFrequencyHzFunc = func() int { return 20 }
	injectImu.PropertiesFunc = func() s.MovementSensorProperties {
		return s.MovementSensorProperties{
			IMUSupported: true,
		}
	}

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:          &injectLidar,
		MovementSensor: &injectImu,
		Timeout:        10 * time.Second,
	}

	t.Run("return error when AddIMUReading errors out", func(t *testing.T) {
		expectedErr := errors.New("failed to get imu reading")
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			return expectedErr
		}

		err := config.tryAddIMUReading(context.Background(), imuReading)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeError, expectedErr)
	})

	t.Run("succeeds when AddIMUReading succeeds", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			return nil
		}

		err := config.tryAddIMUReading(context.Background(), imuReading)
		test.That(t, err, test.ShouldBeNil)
	})
}

func TestTryAddOdometerReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	odometerReading := s.TimedOdometerReadingResponse{
		Position:    geo.NewPoint(5, 4),
		Orientation: &spatialmath.Quaternion{Real: 0.1, Imag: -0.2, Jmag: 2.5, Kmag: -9.1},
		ReadingTime: time.Now().UTC(),
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	injectOdometer := inject.TimedMovementSensor{}
	injectOdometer.NameFunc = func() string { return "good_odometer" }
	injectOdometer.DataFrequencyHzFunc = func() int { return 20 }
	injectOdometer.PropertiesFunc = func() s.MovementSensorProperties {
		return s.MovementSensorProperties{
			OdometerSupported: true,
		}
	}

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:          &injectLidar,
		MovementSensor: &injectOdometer,
		Timeout:        10 * time.Second,
	}

	t.Run("return error when AddOdometerReading errors out", func(t *testing.T) {
		expectedErr := errors.New("failed to get odometer reading")
		cf.AddOdometerReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedOdometerReadingResponse,
		) error {
			return expectedErr
		}

		err := config.tryAddOdometerReading(context.Background(), odometerReading)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeError, expectedErr)
	})

	t.Run("succeeds when AddOdometerReading succeeds", func(t *testing.T) {
		cf.AddOdometerReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedOdometerReadingResponse,
		) error {
			return nil
		}

		err := config.tryAddOdometerReading(context.Background(), odometerReading)
		test.That(t, err, test.ShouldBeNil)
	})
}
