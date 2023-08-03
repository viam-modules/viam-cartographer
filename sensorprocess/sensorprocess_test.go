package sensorprocess

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
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

func TestAddSensorReadingOffline(t *testing.T) {
	logger := golog.NewTestLogger(t)
	reading := []byte("12345")
	readingTimestamp := time.Now().UTC()
	cf := cartofacade.Mock{}
	config := Config{
		Logger:            logger,
		CartoFacade:       &cf,
		LidarName:         "good_lidar",
		LidarDataRateMsec: 200,
		Timeout:           10 * time.Second,
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
		tryAddSensorReadingUntilSuccess(context.Background(), reading, readingTimestamp, config)
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
		tryAddSensorReadingUntilSuccess(cancelCtx, reading, readingTimestamp, config)
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
		tryAddSensorReadingUntilSuccess(cancelCtx, reading, readingTimestamp, config)
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
		tryAddSensorReadingUntilSuccess(cancelCtx, reading, readingTimestamp, config)
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

func TestAddSensorReadingOnline(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}
	reading := []byte("12345")
	readingTimestamp := time.Now().UTC()
	config := Config{
		Logger:            logger,
		CartoFacade:       &cf,
		LidarName:         "good_lidar",
		LidarDataRateMsec: 200,
		Timeout:           10 * time.Second,
	}

	t.Run("When AddSensorReading blocks for more than the DataFreqHz and succeeds, time to sleep is 0", func(t *testing.T) {
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

		timeToSleep := tryAddSensorReading(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("AddSensorReading slower than DataFreqHz and returns lock error, time to sleep is 0", func(t *testing.T) {
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

		timeToSleep := tryAddSensorReading(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("When AddSensorReading blocks for more than the DataFreqHz and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
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

		timeToSleep := tryAddSensorReading(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("AddSensorReading faster than the DataFreqHz and succeeds, time to sleep is <= DataFreqHz", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return nil
		}

		timeToSleep := tryAddSensorReading(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, config.LidarDataRateMsec)
	})

	t.Run("AddSensorReading faster than the DataFreqHz and returns lock error, time to sleep is <= DataFreqHz", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := tryAddSensorReading(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, config.LidarDataRateMsec)
	})

	t.Run("AddSensorReading faster than DataFreqHz and returns unexpected error, time to sleep is <= DataFreqHz", func(t *testing.T) {
		cf.AddSensorReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return errUnknown
		}

		timeToSleep := tryAddSensorReading(context.Background(), reading, readingTimestamp, config)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, config.LidarDataRateMsec)
	})
}

func onlineModeTestHelper(
	ctx context.Context,
	t *testing.T,
	config Config,
	cf cartofacade.Mock,
	cam string,
) {
	logger := golog.NewTestLogger(t)
	onlineSensor, err := s.NewLidar(context.Background(), s.SetupDeps(cam), cam, logger)
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

	config.CartoFacade = &cf
	config.Lidar = onlineSensor
	config.LidarName = onlineSensor.Name
	config.LidarDataRateMsec = 10

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
		test.That(t, call.sensorName, test.ShouldResemble, cam)
		// the lidar test fixture happens to always return the same pcd currently
		// in reality it could be a new pcd every time
		test.That(t, call.currentReading, test.ShouldResemble, expectedPCD)
		test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
	}

	if cam == "good_lidar" {
		test.That(t, calls[0].readingTimestamp.Before(calls[1].readingTimestamp), test.ShouldBeTrue)
		test.That(t, calls[1].readingTimestamp.Before(calls[2].readingTimestamp), test.ShouldBeTrue)
	} else if cam == "replay_lidar" {
		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTime)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, calls[0].readingTimestamp.Equal(readingTime), test.ShouldBeTrue)
	} else {
		t.Errorf("no timestamp tests provided for %v", cam)
	}
}

func invalidSensorTestHelper(
	ctx context.Context,
	t *testing.T,
	cartoFacadeMock cartofacade.Mock,
	config Config,
	cameraName string,
	lidarDataRateMsec int,
) {
	logger := golog.NewTestLogger(t)
	sensor, err := s.NewLidar(context.Background(), s.SetupDeps(cameraName, ""), cameraName, logger)
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
	config.LidarDataRateMsec = lidarDataRateMsec

	jobDone := addSensorReading(ctx, config)
	test.That(t, len(calls), test.ShouldEqual, 0)
	test.That(t, jobDone, test.ShouldBeFalse)
}

func TestAddSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}

	config := Config{
		Logger:            logger,
		CartoFacade:       &cf,
		LidarDataRateMsec: 200,
		Timeout:           10 * time.Second,
	}
	ctx := context.Background()

	t.Run("returns error in online mode when lidar GetData returns error, doesn't try to add sensor data", func(t *testing.T) {
		cam := "invalid_lidar"
		invalidSensorTestHelper(
			ctx,
			t,
			cf,
			config,
			cam,
			10,
		)
	})

	t.Run("returns error in online mode when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		cam := "invalid_replay_lidar"
		invalidSensorTestHelper(
			ctx,
			t,
			cf,
			config,
			cam,
			10,
		)
	})

	t.Run("replay sensor adds sensor data until success in offline mode", func(t *testing.T) {
		cam := "replay_lidar"
		logger := golog.NewTestLogger(t)
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(cam, ""), cam, logger)
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
		config.LidarDataRateMsec = 0

		jobDone := addSensorReading(ctx, config)
		test.That(t, len(calls), test.ShouldEqual, 3)
		test.That(t, jobDone, test.ShouldBeFalse)

		firstTimestamp := calls[0].readingTimestamp
		for i, call := range calls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, "replay_lidar")
			test.That(t, call.currentReading, test.ShouldResemble, expectedPCD)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.readingTimestamp, test.ShouldEqual, firstTimestamp)
		}
	})

	t.Run("online replay lidar adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeTestHelper(ctx, t, config, cf, "replay_lidar")
	})

	t.Run("online lidar adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeTestHelper(ctx, t, config, cf, "good_lidar")
	})

	t.Run("returns true when lidar returns an error that it reached end of dataset", func(t *testing.T) {
		cam := "finished_replay_lidar"
		logger := golog.NewTestLogger(t)
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(cam, ""), cam, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor
		config.LidarDataRateMsec = 0

		jobDone := addSensorReading(ctx, config)
		test.That(t, jobDone, test.ShouldBeTrue)
	})
}

func TestStart(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}

	config := Config{
		Logger:            logger,
		CartoFacade:       &cf,
		LidarDataRateMsec: 200,
		Timeout:           10 * time.Second,
	}
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	t.Run("returns true when lidar returns an error that it reached end of dataset but the context is valid", func(t *testing.T) {
		cam := "finished_replay_lidar"
		logger := golog.NewTestLogger(t)
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(cam, ""), cam, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor
		config.LidarDataRateMsec = 0

		jobDone := Start(context.Background(), config)
		test.That(t, jobDone, test.ShouldBeTrue)
	})

	t.Run("returns false when lidar returns an error that it reached end of dataset but the context was cancelled", func(t *testing.T) {
		cam := "finished_replay_lidar"
		logger := golog.NewTestLogger(t)
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(cam, ""), cam, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor
		config.LidarDataRateMsec = 0

		cancelFunc()

		jobDone := Start(cancelCtx, config)
		test.That(t, jobDone, test.ShouldBeFalse)
	})
}
