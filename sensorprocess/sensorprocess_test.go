package sensorprocess

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
	"github.com/viamrobotics/viam-cartographer/testhelper"
)

type addSensorReadingArgs struct {
	timeout          time.Duration
	sensorName       string
	currentReading   []byte
	readingTimestamp time.Time
}

var (
	expectedPCD = []byte(`VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 0
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 0
DATA binary
`)
	errUnknown = errors.New("unknown error")
)

func TestAddSensorReadingReplaySensor(t *testing.T) {
	logger := golog.NewTestLogger(t)
	reading := []byte("12345")
	readingTimestamp := time.Now().UTC()
	cf := cartofacade.Mock{}
	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
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
		addSensorReadingFromReplaySensor(context.Background(), reading, readingTimestamp, config)
	})

	t.Run("AddSensorReading returns UNABLE_TO_ACQUIRE_LOCK error and the context is cancelled, no infinite loop", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		addSensorReadingFromReplaySensor(cancelCtx, reading, readingTimestamp, config)
	})

	t.Run("When AddSensorReading returns a different error and the context is cancelled, no infinite loop", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return errUnknown
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		addSensorReadingFromReplaySensor(cancelCtx, reading, readingTimestamp, config)
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
				return errUnknown
			}
			if len(calls) < 4 {
				return cartofacade.ErrUnableToAcquireLock
			}
			return nil
		}
		addSensorReadingFromReplaySensor(cancelCtx, reading, readingTimestamp, config)
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
	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		LidarName:   "good_lidar",
		DataRateMs:  200,
		Timeout:     10 * time.Second,
	}

	t.Run("When AddSensorReading blocks for more than the DataRateMs and succeeds, time to sleep is 0", func(t *testing.T) {
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

		timeToSleep := addSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("AddSensorReading slower than DataRateMs and returns lock error, time to sleep is 0", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := addSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("When AddSensorReading blocks for more than the DataRateMs and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return errUnknown
		}

		timeToSleep := addSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("AddSensorReading faster than the DataRateMs and succeeds, time to sleep is <= DataRateMs", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return nil
		}

		timeToSleep := addSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, config.DataRateMs)
	})

	t.Run("AddSensorReading faster than the DataRateMs and returns lock error, time to sleep is <= DataRateMs", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := addSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, config.DataRateMs)
	})

	t.Run("AddSensorReading faster than DataRateMs and returns unexpected error, time to sleep is <= DataRateMs", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return errUnknown
		}

		timeToSleep := addSensorReadingFromLiveReadings(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, config.DataRateMs)
	})
}

func invalidSensorTestHelper(
	ctx context.Context,
	t *testing.T,
	cartoFacadeMock cartofacade.Mock,
	config Config,
	sensors []string,
) {
	sensor, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
	test.That(t, err, test.ShouldBeNil)

	var calls []addSensorReadingArgs
	cartoFacadeMock.AddSensorReadingFunc = func(
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
	config.Lidar = sensor
	config.LidarName = sensor.Name

	jobDone := addSensorReading(ctx, config)
	test.That(t, len(calls), test.ShouldEqual, 0)
	test.That(t, jobDone, test.ShouldBeFalse)
}

func TestAddSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		DataRateMs:  200,
		Timeout:     10 * time.Second,
	}
	ctx := context.Background()

	t.Run("returns error when lidar GetData returns error, doesn't try to add sensor data", func(t *testing.T) {
		sensors := []string{"invalid_sensor"}
		invalidSensorTestHelper(
			ctx,
			t,
			cf,
			config,
			sensors,
		)
	})

	t.Run("returns error when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		sensors := []string{"invalid_replay_sensor"}
		invalidSensorTestHelper(
			ctx,
			t,
			cf,
			config,
			sensors,
		)
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
				return errUnknown
			}
			if len(calls) == 2 {
				return cartofacade.ErrUnableToAcquireLock
			}
			return nil
		}
		config.Lidar = replaySensor
		config.LidarName = replaySensor.Name

		jobDone := addSensorReading(ctx, config)
		test.That(t, len(calls), test.ShouldEqual, 3)
		test.That(t, jobDone, test.ShouldBeFalse)

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
				return errUnknown
			}
			if len(calls) == 2 {
				return cartofacade.ErrUnableToAcquireLock
			}
			return nil
		}
		config.Lidar = liveSensor
		config.LidarName = liveSensor.Name

		jobDone := addSensorReading(ctx, config)
		test.That(t, len(calls), test.ShouldEqual, 1)
		test.That(t, jobDone, test.ShouldBeFalse)

		jobDone = addSensorReading(ctx, config)
		test.That(t, len(calls), test.ShouldEqual, 2)
		test.That(t, jobDone, test.ShouldBeFalse)

		jobDone = addSensorReading(ctx, config)
		test.That(t, len(calls), test.ShouldEqual, 3)
		test.That(t, jobDone, test.ShouldBeFalse)

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

	t.Run("returns true when lidar returns an error that it reached end of dataset", func(t *testing.T) {
		sensors := []string{"finished_replay_sensor"}
		replaySensor, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor

		jobDone := addSensorReading(ctx, config)
		test.That(t, jobDone, test.ShouldBeTrue)
	})
}
