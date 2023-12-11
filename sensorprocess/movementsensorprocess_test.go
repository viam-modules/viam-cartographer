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

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/inject"
)

func TestStartMovementSensor(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	injectImu := inject.TimedMovementSensor{}
	injectImu.NameFunc = func() string { return "good_imu" }
	injectImu.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:          &injectLidar,
		MovementSensor: &injectImu,
		Timeout:        10 * time.Second,
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

	injectImu := inject.TimedMovementSensor{}
	injectImu.NameFunc = func() string { return "good_imu" }
	injectImu.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       true,
		MovementSensor: &injectImu,
		Timeout:        10 * time.Second,
	}

	t.Run("returns error when IMU GetData returns error, doesn't try to add IMU data", func(t *testing.T) {
		invalidOnlineModeIMUTestHelper(context.Background(), t, cf, config, 10, s.IMUWithErroringFunctions, 10)
	})

	t.Run("returns error when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		invalidOnlineModeIMUTestHelper(context.Background(), t, cf, config, 10, s.InvalidReplayIMU, 10)
	})

	t.Run("online replay IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeIMUTestHelper(context.Background(), t, config, cf, s.ReplayIMU)
	})

	t.Run("online IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeIMUTestHelper(context.Background(), t, config, cf, s.GoodIMU)
	})
}

func TestTryAddMovementSensorReadingUntilSuccess(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	cf := cartofacade.Mock{}

	dataFrequencyHz := 0
	injectImu := inject.TimedMovementSensor{}
	injectImu.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       false,
		MovementSensor: &injectImu,
		Timeout:        10 * time.Second,
	}

	t.Run("replay IMU adds sensor data until success", func(t *testing.T) {
		lidar, imu := s.NoLidar, s.ReplayIMU
		replayIMU, err := s.NewMovementSensor(context.Background(), s.SetupDeps(lidar, imu), string(imu), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		injectLidar := inject.TimedLidar{}
		injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

		var calls []addIMUReadingArgs
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			args := addIMUReadingArgs{
				timeout:        timeout,
				sensorName:     sensorName,
				currentReading: currentReading,
			}
			calls = append(calls, args)
			if len(calls) == 1 {
				return errUnknown
			}
			if len(calls) == 2 {
				return cartofacade.ErrUnableToAcquireLock
			}
			return nil
		}
		config.MovementSensor = replayIMU
		config.Lidar = &injectLidar

		config.tryAddMovementSensorReadingUntilSuccess(ctx, expectedMovementSensorReading)
		test.That(t, len(calls), test.ShouldEqual, 3)

		firstTimestamp := calls[0].currentReading.ReadingTime
		for i, call := range calls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, string(imu))
			test.That(t, call.currentReading.LinearAcceleration, test.ShouldResemble, expectedIMUReading.LinearAcceleration)
			test.That(t, call.currentReading.AngularVelocity, test.ShouldResemble, expectedIMUReading.AngularVelocity)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.currentReading.ReadingTime, test.ShouldEqual, firstTimestamp)
		}
	})
}

func TestTryAddMovementSensorReadingOnce(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	imuReading := s.TimedIMUReadingResponse{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 1, Y: 1, Z: 1},
		ReadingTime:        time.Now().UTC(),
	}

	reading := s.TimedMovementSensorReadingResponse{
		TimedIMUResponse: &imuReading,
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

	t.Run("when AddIMUReading blocks for more than the date rate and succeeds, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return nil
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading is slower than date rate and returns a lock error, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading blocks for more than the date rate and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return errUnknown
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading is faster than the date rate and succeeds, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			return nil
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
	})

	t.Run("when AddIMUReading is faster than the date rate and returns a lock error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
	})

	t.Run("when AddIMUReading is faster than date rate "+
		"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			return errUnknown
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
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
