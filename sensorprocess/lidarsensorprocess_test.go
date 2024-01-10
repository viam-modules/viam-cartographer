package sensorprocess

import (
	"context"
	"errors"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/inject"
)

func TestStartLidar(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    true,
		Timeout:     10 * time.Second,
	}

	t.Run("exits loop when the context was cancelled", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		lidar, imu := s.FinishedReplayLidar, s.NoMovementSensor
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), 5, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor

		cancelFunc()

		config.StartLidar(cancelCtx)
	})
}

func TestAddLidarReadingInOnline(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	dataFrequencyHz := 5
	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}

	t.Run("returns error when lidar GetData returns error, doesn't try to add lidar data", func(t *testing.T) {
		invalidAddLidarReadingInOnlineTestHelper(context.Background(), t, cf, config, s.LidarWithErroringFunctions, 10)
	})

	t.Run("returns error when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		invalidAddLidarReadingInOnlineTestHelper(context.Background(), t, cf, config, s.InvalidReplayLidar, 10)
	})

	t.Run("online lidar adds sensor reading once and ignores errors", func(t *testing.T) {
		validAddLidarReadingInOnlineTestHelper(context.Background(), t, config, cf, s.GoodLidar)
	})
}

func TestTryAddLidarReadingUntilSuccess(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	cf.RunFinalOptimizationFunc = func(context.Context, time.Duration) error {
		return nil
	}

	dataFrequencyHz := 0

	lidarReading := s.TimedLidarReadingResponse{
		Reading:     []byte("12345"),
		ReadingTime: time.Now().UTC(),
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    false,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}

	t.Run("replay lidar adds sensor data until success", func(t *testing.T) {
		lidar, imu := s.ReplayLidar, s.NoMovementSensor
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		var calls []addLidarReadingArgs
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			args := addLidarReadingArgs{
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
		config.Lidar = replaySensor

		config.tryAddLidarReadingUntilSuccess(context.Background(), lidarReading)
		test.That(t, len(calls), test.ShouldEqual, 3)

		firstTimestamp := calls[0].currentReading.ReadingTime
		for i, call := range calls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, string(lidar))
			test.That(t, call.currentReading.Reading, test.ShouldResemble, lidarReading.Reading)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.currentReading.ReadingTime, test.ShouldEqual, firstTimestamp)
		}
	})
}

func TestTryAddLidarReadingOnce(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	reading := s.TimedLidarReadingResponse{
		Reading:     []byte("12345"),
		ReadingTime: time.Now().UTC(),
	}

	dataFrequencyHz := 5
	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}
	t.Run("when AddLidarReading blocks for more than the data rate and succeeds, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return nil
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading is slower than data rate and returns a lock error, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading blocks for more than the date rate "+
		"and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return errUnknown
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading is faster than the date rate and succeeds, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			return nil
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.Lidar.DataFrequencyHz())
	})

	t.Run("when AddLidarReading is faster than the date rate "+
		"and returns lock error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.Lidar.DataFrequencyHz())
	})

	t.Run("when AddLidarReading is faster than date rate "+
		"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			return errUnknown
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.Lidar.DataFrequencyHz())
	})
}

func TestTryAddLidarReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	reading := s.TimedLidarReadingResponse{
		Reading:     []byte("12345"),
		ReadingTime: time.Now().UTC(),
	}

	dataFrequencyHz := 5
	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}
	t.Run("return error when AddLidarReading errors out", func(t *testing.T) {
		expectedErr := errors.New("failed to get lidar reading")
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			return expectedErr
		}

		err := config.tryAddLidarReading(context.Background(), reading)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeError, expectedErr)
	})

	t.Run("succeeds when AddLidarReading succeeds", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			return nil
		}

		err := config.tryAddLidarReading(context.Background(), reading)
		test.That(t, err, test.ShouldBeNil)
	})
}
