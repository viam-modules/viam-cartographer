package sensorprocess

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
)

type addLidarReadingArgs struct {
	timeout          time.Duration
	sensorName       string
	currentReading   []byte
	readingTimestamp time.Time
}

type addIMUReadingArgs struct {
	timeout          time.Duration
	sensorName       string
	currentReading   cartofacade.IMUReading
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
	expectedIMUReading = cartofacade.IMUReading{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 0.017453292519943295, Y: 0.008726646259971648, Z: 0},
	}
	errUnknown = errors.New("unknown error")
)

func TestAddLidarReadingOffline(t *testing.T) {
	logger := golog.NewTestLogger(t)
	reading := []byte("12345")
	readingTimestamp := time.Now().UTC()
	cf := cartofacade.Mock{}

	runFinalOptimizationFunc := func(context.Context, time.Duration) error {
		return nil
	}
	config := Config{
		Logger:                   logger,
		CartoFacade:              &cf,
		LidarName:                "good_lidar",
		LidarDataFrequencyHz:     5,
		Timeout:                  10 * time.Second,
		RunFinalOptimizationFunc: runFinalOptimizationFunc,
	}

	t.Run("success, no infinite loop", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return nil
		}
		config.tryAddLidarReadingUntilSuccess(context.Background(), reading, readingTimestamp)
	})

	t.Run("failure with UNABLE_TO_ACQUIRE_LOCK error and cancelled context, no infinite loop", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
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
		config.tryAddLidarReadingUntilSuccess(cancelCtx, reading, readingTimestamp)
	})

	t.Run("failure with a different error and cancelled context, no infinite loop", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
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
		config.tryAddLidarReadingUntilSuccess(cancelCtx, reading, readingTimestamp)
	})

	t.Run("failure with errors being hit a few times, a retry, and then success", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		var calls []addLidarReadingArgs

		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			args := addLidarReadingArgs{
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
		config.tryAddLidarReadingUntilSuccess(cancelCtx, reading, readingTimestamp)
		test.That(t, len(calls), test.ShouldEqual, 4)
		for i, args := range calls {
			t.Logf("addLidarReadingArgsHistory %d", i)
			test.That(t, args.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, args.sensorName, test.ShouldEqual, config.LidarName)
			test.That(t, args.currentReading, test.ShouldResemble, reading)
			test.That(t, args.readingTimestamp, test.ShouldResemble, readingTimestamp)
		}
		cancelFunc()
	})
}

func TestAddIMUReadingOffline(t *testing.T) {
	logger := golog.NewTestLogger(t)
	reading := cartofacade.IMUReading{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 1, Y: 1, Z: 1},
	}
	readingTimestamp := time.Now().UTC()
	cf := cartofacade.Mock{}
	config := Config{
		Logger:             logger,
		CartoFacade:        &cf,
		IMUName:            "good_imu",
		IMUDataFrequencyHz: 20,
		Timeout:            10 * time.Second,
	}
	t.Run("success, no infinite loop", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			return nil
		}
		config.tryAddIMUReadingUntilSuccess(context.Background(), reading, readingTimestamp)
	})

	t.Run("failure with UNABLE_TO_ACQUIRE_LOCK error and cancelled context, no infinite loop", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		config.tryAddIMUReadingUntilSuccess(cancelCtx, reading, readingTimestamp)
	})

	t.Run("failure with a different error and cancelled context, no infinite loop", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			return errUnknown
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		config.tryAddIMUReadingUntilSuccess(cancelCtx, reading, readingTimestamp)
	})

	t.Run("failure with errors being hit a few times, a retry, and then success", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		var calls []addIMUReadingArgs

		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			args := addIMUReadingArgs{
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
		config.tryAddIMUReadingUntilSuccess(cancelCtx, reading, readingTimestamp)
		test.That(t, len(calls), test.ShouldEqual, 4)
		for i, args := range calls {
			t.Logf("addIMUReadingArgsHistory %d", i)
			test.That(t, args.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, args.sensorName, test.ShouldEqual, config.IMUName)
			test.That(t, args.currentReading, test.ShouldResemble, reading)
			test.That(t, args.readingTimestamp, test.ShouldResemble, readingTimestamp)
		}
		cancelFunc()
	})
}

func TestAddLidarReadingOnline(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}
	reading := []byte("12345")
	readingTimestamp := time.Now().UTC()
	config := Config{
		Logger:                   logger,
		CartoFacade:              &cf,
		LidarName:                "good_lidar",
		LidarDataFrequencyHz:     5,
		Timeout:                  10 * time.Second,
		RunFinalOptimizationFunc: cf.RunFinalOptimization,
	}

	t.Run("when AddLidarReading blocks for more than the data rate and succeeds, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return nil
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading is slower than data rate and returns a lock error, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading blocks for more than the date rate "+
		"and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return errUnknown
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading is faster than the date rate and succeeds, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return nil
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.LidarDataFrequencyHz)
	})

	t.Run("when AddLidarReading is faster than the date rate "+
		"and returns lock error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.LidarDataFrequencyHz)
	})

	t.Run("when AddLidarReading is faster than date rate "+
		"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			return errUnknown
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.LidarDataFrequencyHz)
	})
}

func TestAddIMUReadingOnline(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}
	reading := cartofacade.IMUReading{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 1, Y: 1, Z: 1},
	}
	readingTimestamp := time.Now().UTC()
	config := Config{
		Logger:               logger,
		CartoFacade:          &cf,
		LidarDataFrequencyHz: 5,
		IMUName:              "good_imu",
		IMUDataFrequencyHz:   20,
		Timeout:              10 * time.Second,
	}

	t.Run("when AddIMUReading blocks for more than the date rate and succeeds, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return nil
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading is slower than date rate and returns a lock error, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading blocks for more than the date rate and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			time.Sleep(1 * time.Second)
			return errUnknown
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading is faster than the date rate and succeeds, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			return nil
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.IMUDataFrequencyHz)
	})

	t.Run("when AddIMUReading is faster than the date rate and returns a lock error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.IMUDataFrequencyHz)
	})

	t.Run("when AddIMUReading is faster than date rate "+
		"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			return errUnknown
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading, readingTimestamp)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.IMUDataFrequencyHz)
	})
}

func onlineModeLidarTestHelper(
	ctx context.Context,
	t *testing.T,
	config Config,
	cf cartofacade.Mock,
	cameraName string,
) {
	logger := golog.NewTestLogger(t)
	dataFrequencyHz := 100
	onlineSensor, err := s.NewLidar(context.Background(), s.SetupDeps(cameraName, ""), cameraName, dataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var calls []addLidarReadingArgs
	cf.AddLidarReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading []byte,
		readingTimestamp time.Time,
	) error {
		args := addLidarReadingArgs{
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
	config.LidarName = onlineSensor.Name()
	config.LidarDataFrequencyHz = dataFrequencyHz

	jobDone := config.addLidarReading(ctx)
	test.That(t, len(calls), test.ShouldEqual, 1)
	test.That(t, jobDone, test.ShouldBeFalse)

	jobDone = config.addLidarReading(ctx)
	test.That(t, len(calls), test.ShouldEqual, 2)
	test.That(t, jobDone, test.ShouldBeFalse)

	jobDone = config.addLidarReading(ctx)
	test.That(t, len(calls), test.ShouldEqual, 3)
	test.That(t, jobDone, test.ShouldBeFalse)

	for i, call := range calls {
		t.Logf("call %d", i)
		test.That(t, call.sensorName, test.ShouldResemble, cameraName)
		// the lidar test fixture happens to always return the same pcd currently
		// in reality it could be a new pcd every time
		test.That(t, call.currentReading, test.ShouldResemble, expectedPCD)
		test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
	}

	if cameraName == "good_lidar" {
		test.That(t, calls[0].readingTimestamp.Before(calls[1].readingTimestamp), test.ShouldBeTrue)
		test.That(t, calls[1].readingTimestamp.Before(calls[2].readingTimestamp), test.ShouldBeTrue)
	} else if cameraName == "replay_lidar" {
		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTimestamp)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, calls[0].readingTimestamp.Equal(readingTime), test.ShouldBeTrue)
	} else {
		t.Errorf("no timestamp tests provided for %v", cameraName)
	}
}

func onlineModeIMUTestHelper(
	ctx context.Context,
	t *testing.T,
	config Config,
	cf cartofacade.Mock,
	movementSensorName string,
) {
	logger := golog.NewTestLogger(t)
	dataFrequencyHz := 100
	onlineIMU, err := s.NewIMU(context.Background(), s.SetupDeps("", movementSensorName), movementSensorName, dataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var calls []addIMUReadingArgs
	cf.AddIMUReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading cartofacade.IMUReading,
		readingTimestamp time.Time,
	) error {
		args := addIMUReadingArgs{
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
	config.IMU = onlineIMU
	config.IMUName = onlineIMU.Name()
	config.IMUDataFrequencyHz = dataFrequencyHz

	// set lidar data rate to signify that we are in online mode
	config.LidarDataFrequencyHz = 100
	config.currentLidarData.time = time.Now().UTC().Add(-10 * time.Second)
	config.sensorProcessStartTime = time.Time{}.Add(time.Millisecond)

	jobDone := config.addIMUReading(ctx)
	test.That(t, len(calls), test.ShouldEqual, 1)
	test.That(t, jobDone, test.ShouldBeFalse)

	jobDone = config.addIMUReading(ctx)
	test.That(t, len(calls), test.ShouldEqual, 2)
	test.That(t, jobDone, test.ShouldBeFalse)

	jobDone = config.addIMUReading(ctx)
	test.That(t, len(calls), test.ShouldEqual, 3)
	test.That(t, jobDone, test.ShouldBeFalse)

	for i, call := range calls {
		t.Logf("call %d", i)
		test.That(t, call.sensorName, test.ShouldResemble, movementSensorName)
		// the IMU test fixture happens to always return the same readings currently
		// in reality they are likely different every time
		test.That(t, call.currentReading, test.ShouldResemble, expectedIMUReading)
		test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
	}

	if movementSensorName == "good_imu" {
		test.That(t, calls[0].readingTimestamp.Before(calls[1].readingTimestamp), test.ShouldBeTrue)
		test.That(t, calls[1].readingTimestamp.Before(calls[2].readingTimestamp), test.ShouldBeTrue)
	} else if movementSensorName == "replay_imu" {
		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTimestamp)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, calls[0].readingTimestamp.Equal(readingTime), test.ShouldBeTrue)
	} else {
		t.Errorf("no timestamp tests provided for %v", movementSensorName)
	}
}

func invalidLidarTestHelper(
	ctx context.Context,
	t *testing.T,
	cartoFacadeMock cartofacade.Mock,
	config Config,
	cameraName string,
	lidarDataFrequencyHz int,
) {
	logger := golog.NewTestLogger(t)
	lidar, err := s.NewLidar(context.Background(), s.SetupDeps(cameraName, ""), cameraName, lidarDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var calls []addLidarReadingArgs
	cartoFacadeMock.AddLidarReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading []byte,
		readingTimestamp time.Time,
	) error {
		args := addLidarReadingArgs{
			timeout:          timeout,
			sensorName:       sensorName,
			currentReading:   currentReading,
			readingTimestamp: readingTimestamp,
		}
		calls = append(calls, args)
		return nil
	}
	config.Lidar = lidar
	config.LidarName = lidar.Name()
	config.LidarDataFrequencyHz = lidarDataFrequencyHz

	jobDone := config.addLidarReading(ctx)
	test.That(t, len(calls), test.ShouldEqual, 0)
	test.That(t, jobDone, test.ShouldBeFalse)
}

func invalidIMUTestHelper(
	ctx context.Context,
	t *testing.T,
	cartoFacadeMock cartofacade.Mock,
	config Config,
	lidarDataFrequencyHz int,
	movementSensorName string,
	imuDataFrequencyHz int,
) {
	logger := golog.NewTestLogger(t)
	imu, err := s.NewIMU(context.Background(), s.SetupDeps("", movementSensorName), movementSensorName, lidarDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var calls []addIMUReadingArgs
	cartoFacadeMock.AddIMUReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading cartofacade.IMUReading,
		readingTimestamp time.Time,
	) error {
		args := addIMUReadingArgs{
			timeout:          timeout,
			sensorName:       sensorName,
			currentReading:   currentReading,
			readingTimestamp: readingTimestamp,
		}
		calls = append(calls, args)
		return nil
	}
	config.LidarDataFrequencyHz = lidarDataFrequencyHz
	config.IMU = imu
	config.IMUName = imu.Name()
	config.IMUDataFrequencyHz = imuDataFrequencyHz

	jobDone := config.addIMUReading(ctx)
	test.That(t, len(calls), test.ShouldEqual, 0)
	test.That(t, jobDone, test.ShouldBeFalse)
}

func TestAddLidarReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}

	runFinalOptimizationFunc := func(context.Context, time.Duration) error {
		return nil
	}

	config := Config{
		Logger:                   logger,
		CartoFacade:              &cf,
		LidarDataFrequencyHz:     5,
		Timeout:                  10 * time.Second,
		RunFinalOptimizationFunc: runFinalOptimizationFunc,
		Mutex:                    &sync.Mutex{},
	}
	ctx := context.Background()

	t.Run("returns error in online mode when lidar GetData returns error, doesn't try to add lidar data", func(t *testing.T) {
		cameraName := "lidar_with_erroring_functions"
		invalidLidarTestHelper(
			ctx,
			t,
			cf,
			config,
			cameraName,
			10,
		)
	})

	t.Run("returns error in online mode when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		cameraName := "invalid_replay_lidar"
		invalidLidarTestHelper(
			ctx,
			t,
			cf,
			config,
			cameraName,
			10,
		)
	})

	t.Run("replay sensor adds sensor data until success in offline mode", func(t *testing.T) {
		cameraName := "replay_lidar"
		logger := golog.NewTestLogger(t)
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(cameraName, ""), cameraName, dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		var calls []addLidarReadingArgs
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading []byte,
			readingTimestamp time.Time,
		) error {
			args := addLidarReadingArgs{
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
		config.LidarName = replaySensor.Name()
		config.LidarDataFrequencyHz = dataFrequencyHz

		_ = config.addLidarReading(ctx) // first call gets data
		jobDone := config.addLidarReading(ctx)
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
		onlineModeLidarTestHelper(ctx, t, config, cf, "replay_lidar")
	})

	t.Run("online lidar adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeLidarTestHelper(ctx, t, config, cf, "good_lidar")
	})

	t.Run("returns true when lidar returns an error that it reached end of dataset and optimization function succeeds", func(t *testing.T) {
		cameraName := "finished_replay_lidar"
		logger := golog.NewTestLogger(t)
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(cameraName, ""), cameraName, dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor
		config.LidarDataFrequencyHz = dataFrequencyHz

		jobDone := config.addLidarReading(ctx)
		test.That(t, jobDone, test.ShouldBeTrue)
	})

	t.Run("returns true when lidar returns an error that it reached end of dataset and optimization function fails", func(t *testing.T) {
		runFinalOptimizationFunc = func(context.Context, time.Duration) error {
			return errors.New("test error")
		}
		config.RunFinalOptimizationFunc = runFinalOptimizationFunc

		cameraName := "finished_replay_lidar"
		logger := golog.NewTestLogger(t)
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(cameraName, ""), cameraName, dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor
		config.LidarDataFrequencyHz = dataFrequencyHz

		jobDone := config.addLidarReading(ctx)
		test.That(t, jobDone, test.ShouldBeTrue)
	})
}

func TestAddIMUReading(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}

	config := Config{
		Logger:             logger,
		CartoFacade:        &cf,
		IMUDataFrequencyHz: 20,
		Timeout:            10 * time.Second,
		Mutex:              &sync.Mutex{},
	}
	ctx := context.Background()

	t.Run("returns error in online mode when IMU GetData returns error, doesn't try to add IMU data", func(t *testing.T) {
		movementsensor := "imu_with_erroring_functions"
		invalidIMUTestHelper(
			ctx,
			t,
			cf,
			config,
			10,
			movementsensor,
			10,
		)
	})

	t.Run("returns error in online mode when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		movementsensor := "invalid_replay_imu"
		invalidIMUTestHelper(
			ctx,
			t,
			cf,
			config,
			10,
			movementsensor,
			10,
		)
	})

	t.Run("replay sensor adds IMU data until success in offline mode", func(t *testing.T) {
		movementsensor := "replay_imu"
		logger := golog.NewTestLogger(t)
		dataFrequencyHz := 0
		replayIMU, err := s.NewIMU(context.Background(), s.SetupDeps("", movementsensor), movementsensor, dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		var calls []addIMUReadingArgs
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading cartofacade.IMUReading,
			readingTimestamp time.Time,
		) error {
			args := addIMUReadingArgs{
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
		config.IMU = replayIMU
		config.IMUName = replayIMU.Name()
		config.IMUDataFrequencyHz = 0
		config.LidarDataFrequencyHz = dataFrequencyHz
		config.currentLidarData.time = time.Now().UTC().Add(-10 * time.Second)
		config.sensorProcessStartTime = time.Time{}.Add(time.Millisecond)

		_ = config.addIMUReading(ctx) // first call gets data
		jobDone := config.addIMUReading(ctx)
		test.That(t, len(calls), test.ShouldEqual, 3)
		test.That(t, jobDone, test.ShouldBeFalse)

		firstTimestamp := calls[0].readingTimestamp
		for i, call := range calls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, "replay_imu")
			test.That(t, call.currentReading, test.ShouldResemble, expectedIMUReading)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.readingTimestamp, test.ShouldEqual, firstTimestamp)
		}
	})

	t.Run("online replay IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeIMUTestHelper(ctx, t, config, cf, "replay_imu")
	})

	t.Run("online IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeIMUTestHelper(ctx, t, config, cf, "good_imu")
	})

	t.Run("returns true when IMU returns an error that it reached end of dataset", func(t *testing.T) {
		movementsensor := "finished_replay_imu"
		logger := golog.NewTestLogger(t)
		dataFrequencyHz := 0
		replayIMU, err := s.NewIMU(context.Background(), s.SetupDeps("", movementsensor), movementsensor, dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.IMU = replayIMU
		config.IMUDataFrequencyHz = dataFrequencyHz
		config.currentLidarData.time = time.Now().UTC().Add(-10 * time.Second)

		jobDone := config.addIMUReading(ctx)
		test.That(t, jobDone, test.ShouldBeTrue)
	})
}

func TestStartLidar(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}

	config := Config{
		Logger:                   logger,
		CartoFacade:              &cf,
		LidarDataFrequencyHz:     5,
		Timeout:                  10 * time.Second,
		RunFinalOptimizationFunc: cf.RunFinalOptimization,
		Mutex:                    &sync.Mutex{},
	}
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	t.Run("returns true when lidar returns an error that it reached end of dataset but the context is valid", func(t *testing.T) {
		cameraName := "finished_replay_lidar"
		logger := golog.NewTestLogger(t)
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(cameraName, ""), cameraName, dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor
		config.LidarDataFrequencyHz = dataFrequencyHz

		jobDone := config.StartLidar(context.Background())
		test.That(t, jobDone, test.ShouldBeTrue)
	})

	t.Run("returns false when lidar returns an error that it reached end of dataset but the context was cancelled", func(t *testing.T) {
		cameraName := "finished_replay_lidar"
		logger := golog.NewTestLogger(t)
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(cameraName, ""), cameraName, dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor
		config.LidarDataFrequencyHz = dataFrequencyHz

		cancelFunc()

		jobDone := config.StartLidar(cancelCtx)
		test.That(t, jobDone, test.ShouldBeFalse)
	})
}

func TestStartIMU(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}

	config := Config{
		Logger:             logger,
		CartoFacade:        &cf,
		IMUDataFrequencyHz: 20,
		Timeout:            10 * time.Second,
		Mutex:              &sync.Mutex{},
	}
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	t.Run("returns true when IMU returns an error that it reached end of dataset but the context is valid", func(t *testing.T) {
		movementSensorName := "finished_replay_imu"
		logger := golog.NewTestLogger(t)
		dataFrequencyHz := 0
		replaySensor, err := s.NewIMU(context.Background(), s.SetupDeps("", movementSensorName), movementSensorName, dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.IMU = replaySensor
		config.IMUDataFrequencyHz = dataFrequencyHz
		config.currentLidarData.time = time.Now().UTC().Add(-10 * time.Second)

		jobDone := config.StartIMU(context.Background())
		test.That(t, jobDone, test.ShouldBeTrue)
	})

	t.Run("returns false when IMU returns an error that it reached end of dataset but the context was cancelled", func(t *testing.T) {
		movementSensorName := "finished_replay_imu"
		logger := golog.NewTestLogger(t)
		dataFrequencyHz := 0
		replaySensor, err := s.NewIMU(context.Background(), s.SetupDeps("", movementSensorName), movementSensorName, dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.IMU = replaySensor
		config.IMUDataFrequencyHz = dataFrequencyHz
		config.currentLidarData.time = time.Now().UTC().Add(-10 * time.Second)

		cancelFunc()

		jobDone := config.StartLidar(cancelCtx)
		test.That(t, jobDone, test.ShouldBeFalse)
	})
}
