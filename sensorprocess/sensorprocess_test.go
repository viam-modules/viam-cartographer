package sensorprocess

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
)

type addSensorReadingArgs struct {
	timeout          time.Duration
	sensorName       string
	currentReading   []byte
	readingTimestamp time.Time
}

var unknownErr error = errors.New("unknown error")

func TestAddSensorReadingReplaySensor(t *testing.T) {
	logger := golog.NewTestLogger(t)
	reading := []byte("12345")
	readingTimestamp := time.Now().UTC()
	cf := cartofacade.Mock{}
	config := Config{
		Logger:      logger,
		Cartofacade: &cf,
		LidarName:   "good_lidar",
		DataRateMs:  200,
		Timeout:     10 * time.Second,
	}
	t.Run("When addSensorReading returns successfully, no infinite loop", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return nil
		}
		AddSensorReadingFromReplaySensor(context.Background(), reading, readingTimestamp, config)
	})

	t.Run("When AddSensorReading returns UNABLE_TO_ACQUIRE_LOCK error and the context is cancelled, no infinite loop", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return cartofacade.UNABLE_TO_ACQUIRE_LOCK
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		AddSensorReadingFromReplaySensor(cancelCtx, reading, readingTimestamp, config)
	})

	t.Run("When AddSensorReading returns a different error and the context is cancelled, no infinite loop", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return errors.New("non UNABLE_TO_ACQUIRE_LOCK error")
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		AddSensorReadingFromReplaySensor(cancelCtx, reading, readingTimestamp, config)
	})

	t.Run("When AddSensorReading hits errors a few times, retries, and then succeeds", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		var calls []addSensorReadingArgs

		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			args := addSensorReadingArgs{
				timeout:          timeout,
				sensorName:       sensorName,
				currentReading:   currentReading,
				readingTimestamp: readingTimestamp,
			}
			calls = append(calls, args)
			if len(calls) == 1 {
				return errors.New("unexpected error")
			}
			if len(calls) < 4 {
				return cartofacade.UNABLE_TO_ACQUIRE_LOCK
			}
			return nil
		}
		AddSensorReadingFromReplaySensor(cancelCtx, reading, readingTimestamp, config)
		test.That(t, len(calls), test.ShouldEqual, 4)
		for i, args := range calls {
			t.Logf("addSensorReadingArgsHistory %d", i)
			test.That(t, args.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, args.sensorName, test.ShouldEqual, config.LidarName)
			test.That(t, args.currentReading, test.ShouldResemble, reading)
			test.That(t, args.readingTimestamp, test.ShouldResemble, readingTimestamp)
		}
		cancelFunc()
	})
}

func TestAddSensorReadingLiveReadings(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}
	reading := []byte("12345")
	readingTimestamp := time.Now().UTC()
	// When addSensorReading blocks for longer than the data rate
	config := Config{
		Logger:      logger,
		Cartofacade: &cf,
		LidarName:   "good_lidar",
		DataRateMs:  200,
		Timeout:     10 * time.Second,
	}

	t.Run("When AddSensorReading blocks for more than the DataRateMs and succeeds, returns 0", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return nil
		}

		timeToSleep := AddSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("When AddSensorReading blocks for more than the DataRateMs and returns an UNABLE_TO_ACQUIRE_LOCK error, returns 0", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.UNABLE_TO_ACQUIRE_LOCK
		}

		timeToSleep := AddSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("When AddSensorReading blocks for more than the DataRateMs and returns an unexpected error, returns 0", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return errors.New("not UNABLE_TO_ACQUIRE_LOCK error")
		}

		timeToSleep := AddSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("When AddSensorReading blocks for is less than the DataRateMs and succeeds, returns a number less than or equal to the DataRateMs", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return nil
		}

		timeToSleep := AddSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, config.DataRateMs)
	})

	t.Run("When AddSensorReading blocks for is less than the DataRateMs and returns an UNABLE_TO_ACQUIRE_LOCK error, returns a number less than or equal to the DataRateMs", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return cartofacade.UNABLE_TO_ACQUIRE_LOCK
		}

		timeToSleep := AddSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, config.DataRateMs)
	})

	t.Run("When AddSensorReading blocks for is less than the DataRateMs and returns an unexpected error, returns a number less than or equal to the DataRateMs", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return errors.New("not UNABLE_TO_ACQUIRE_LOCK error")
		}

		timeToSleep := AddSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, config.DataRateMs)
	})
}

func TestAddSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}

	config := Config{
		Logger:      logger,
		Cartofacade: &cf,
		DataRateMs:  200,
		Timeout:     10 * time.Second,
	}
	ctx := context.Background()

	t.Run("returns error when lidar GetData returns error, doesn't try to add sensor data", func(t *testing.T) {
		sensors := []string{"invalid_sensor"}
		invalidReplaySensor, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		var calls []addSensorReadingArgs
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			args := addSensorReadingArgs{
				timeout:          timeout,
				sensorName:       sensorName,
				currentReading:   currentReading,
				readingTimestamp: readingTimestamp,
			}
			calls = append(calls, args)
			return nil
		}
		config.Lidar = invalidReplaySensor
		config.LidarName = invalidReplaySensor.Name

		err = AddSensorReading(ctx, config)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, "invalid sensor")
		test.That(t, len(calls), test.ShouldEqual, 0)
	})

	t.Run("returns error when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		sensors := []string{"invalid_replay_sensor"}
		invalidReplaySensor, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		var calls []addSensorReadingArgs
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			args := addSensorReadingArgs{
				timeout:          timeout,
				sensorName:       sensorName,
				currentReading:   currentReading,
				readingTimestamp: readingTimestamp,
			}
			calls = append(calls, args)
			return nil
		}
		config.Lidar = invalidReplaySensor
		config.LidarName = invalidReplaySensor.Name

		err = AddSensorReading(ctx, config)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, "cannot parse")

		test.That(t, len(calls), test.ShouldEqual, 0)
	})

	t.Run("replay sensor adds sensor data until success", func(t *testing.T) {
		sensors := []string{"replay_sensor"}
		replaySensor, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		var calls []addSensorReadingArgs
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			args := addSensorReadingArgs{
				timeout:          timeout,
				sensorName:       sensorName,
				currentReading:   currentReading,
				readingTimestamp: readingTimestamp,
			}
			calls = append(calls, args)
			if len(calls) == 1 {
				return unknownErr
			}
			if len(calls) == 2 {
				return cartofacade.UNABLE_TO_ACQUIRE_LOCK
			}
			return nil
		}
		config.Lidar = replaySensor
		config.LidarName = replaySensor.Name

		err = AddSensorReading(ctx, config)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(calls), test.ShouldEqual, 3)

		expectedPCD := []byte("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\nDATA binary\n")
		firstTimestamp := calls[0].readingTimestamp
		for i, call := range calls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, "replay_sensor")
			test.That(t, call.currentReading, test.ShouldResemble, expectedPCD)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.readingTimestamp, test.ShouldEqual, firstTimestamp)
		}
	})

	t.Run("live sensor adds sensor reading once and ignores errors", func(t *testing.T) {
		sensors := []string{"good_lidar"}
		liveSensor, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		var calls []addSensorReadingArgs
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			args := addSensorReadingArgs{
				timeout:          timeout,
				sensorName:       sensorName,
				currentReading:   currentReading,
				readingTimestamp: readingTimestamp,
			}
			calls = append(calls, args)
			if len(calls) == 1 {
				return unknownErr
			}
			if len(calls) == 2 {
				return cartofacade.UNABLE_TO_ACQUIRE_LOCK
			}
			return nil
		}
		config.Lidar = liveSensor
		config.LidarName = liveSensor.Name

		err = AddSensorReading(ctx, config)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(calls), test.ShouldEqual, 1)

		err = AddSensorReading(ctx, config)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(calls), test.ShouldEqual, 2)

		err = AddSensorReading(ctx, config)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(calls), test.ShouldEqual, 3)

		expectedPCD := []byte("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 0\nDATA binary\n")
		for i, call := range calls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, "good_lidar")
			// the lidar test fixture happens to always return the same pcd currently
			// in reality it could be a new pcd every time
			test.That(t, call.currentReading, test.ShouldResemble, expectedPCD)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
		}
		test.That(t, calls[0].readingTimestamp.Before(calls[1].readingTimestamp), test.ShouldBeTrue)
		test.That(t, calls[1].readingTimestamp.Before(calls[2].readingTimestamp), test.ShouldBeTrue)
	})
}
