package sensorprocess

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/inject"
)

type addLidarReadingArgs struct {
	timeout        time.Duration
	sensorName     string
	currentReading s.TimedLidarSensorReadingResponse
}

type addIMUReadingArgs struct {
	timeout        time.Duration
	sensorName     string
	currentReading s.TimedIMUSensorReadingResponse
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
	expectedIMUReading = s.TimedIMUSensorReadingResponse{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 0.017453292519943295, Y: 0.008726646259971648, Z: 0},
	}
	errUnknown = errors.New("unknown error")
)

func TestAddLidarReadingOffline(t *testing.T) {
	logger := logging.NewTestLogger(t)
	reading := s.TimedLidarSensorReadingResponse{
		Reading:     []byte("12345"),
		ReadingTime: time.Now().UTC(),
	}
	cf := cartofacade.Mock{}

	runFinalOptimizationFunc := func(context.Context, time.Duration) error {
		return nil
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	config := Config{
		Logger:                   logger,
		CartoFacade:              &cf,
		Online:                   injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:                    &injectLidar,
		Timeout:                  10 * time.Second,
		RunFinalOptimizationFunc: runFinalOptimizationFunc,
	}

	t.Run("success, no infinite loop", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
		) error {
			return nil
		}
		config.tryAddLidarReadingUntilSuccess(context.Background(), reading)
	})

	t.Run("failure with UNABLE_TO_ACQUIRE_LOCK error and cancelled context, no infinite loop", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		config.tryAddLidarReadingUntilSuccess(cancelCtx, reading)
	})

	t.Run("failure with a different error and cancelled context, no infinite loop", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
		) error {
			return errUnknown
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		config.tryAddLidarReadingUntilSuccess(cancelCtx, reading)
	})

	t.Run("failure with errors being hit a few times, a retry, and then success", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		var calls []addLidarReadingArgs

		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
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
			if len(calls) < 4 {
				return cartofacade.ErrUnableToAcquireLock
			}
			return nil
		}
		config.tryAddLidarReadingUntilSuccess(cancelCtx, reading)
		test.That(t, len(calls), test.ShouldEqual, 4)
		for i, args := range calls {
			t.Logf("addLidarReadingArgsHistory %d", i)
			test.That(t, args.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, args.sensorName, test.ShouldEqual, config.Lidar.Name())
			test.That(t, args.currentReading, test.ShouldResemble, reading)
		}
		cancelFunc()
	})
}

func TestAddIMUReadingOffline(t *testing.T) {
	logger := logging.NewTestLogger(t)
	reading := s.TimedIMUSensorReadingResponse{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 1, Y: 1, Z: 1},
		ReadingTime:        time.Now().UTC(),
	}
	cf := cartofacade.Mock{}

	injectImu := inject.TimedMovementSensor{}
	injectImu.NameFunc = func() string { return "good_imu" }
	injectImu.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		Online:      false,
		IMU:         &injectImu,
		Timeout:     10 * time.Second,
	}
	t.Run("success, no infinite loop", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
		) error {
			return nil
		}
		config.tryAddIMUReadingUntilSuccess(context.Background(), reading)
	})

	t.Run("failure with UNABLE_TO_ACQUIRE_LOCK error and cancelled context, no infinite loop", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		config.tryAddIMUReadingUntilSuccess(cancelCtx, reading)
	})

	t.Run("failure with a different error and cancelled context, no infinite loop", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
		) error {
			return errUnknown
		}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()
		config.tryAddIMUReadingUntilSuccess(cancelCtx, reading)
	})

	t.Run("failure with errors being hit a few times, a retry, and then success", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		var calls []addIMUReadingArgs

		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
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
			if len(calls) < 4 {
				return cartofacade.ErrUnableToAcquireLock
			}
			return nil
		}
		config.tryAddIMUReadingUntilSuccess(cancelCtx, reading)
		test.That(t, len(calls), test.ShouldEqual, 4)
		for i, args := range calls {
			t.Logf("addIMUReadingArgsHistory %d", i)
			test.That(t, args.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, args.sensorName, test.ShouldEqual, config.IMU.Name())
			test.That(t, args.currentReading, test.ShouldResemble, reading)
		}
		cancelFunc()
	})
}

func TestAddLidarReadingOnline(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	reading := s.TimedLidarSensorReadingResponse{
		Reading:     []byte("12345"),
		ReadingTime: time.Now().UTC(),
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	config := Config{
		Logger:                   logger,
		CartoFacade:              &cf,
		Online:                   injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:                    &injectLidar,
		Timeout:                  10 * time.Second,
		RunFinalOptimizationFunc: cf.RunFinalOptimization,
	}

	t.Run("when AddLidarReading blocks for more than the data rate and succeeds, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return nil
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading is slower than data rate and returns a lock error, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading blocks for more than the date rate "+
		"and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return errUnknown
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading is faster than the date rate and succeeds, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
		) error {
			return nil
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.Lidar.DataFrequencyHz())
	})

	t.Run("when AddLidarReading is faster than the date rate "+
		"and returns lock error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.Lidar.DataFrequencyHz())
	})

	t.Run("when AddLidarReading is faster than date rate "+
		"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
		) error {
			return errUnknown
		}

		timeToSleep := config.tryAddLidarReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.Lidar.DataFrequencyHz())
	})
}

func TestAddIMUReadingOnline(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	reading := s.TimedIMUSensorReadingResponse{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 1, Y: 1, Z: 1},
		ReadingTime:        time.Now().UTC(),
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	injectImu := inject.TimedMovementSensor{}
	injectImu.NameFunc = func() string { return "good_imu" }
	injectImu.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		Online:      injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		IMU:         &injectImu,
		Timeout:     10 * time.Second,
	}

	t.Run("when AddIMUReading blocks for more than the date rate and succeeds, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return nil
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading is slower than date rate and returns a lock error, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading blocks for more than the date rate and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return errUnknown
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading is faster than the date rate and succeeds, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
		) error {
			return nil
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.IMU.DataFrequencyHz())
	})

	t.Run("when AddIMUReading is faster than the date rate and returns a lock error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.IMU.DataFrequencyHz())
	})

	t.Run("when AddIMUReading is faster than date rate "+
		"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
		) error {
			return errUnknown
		}

		timeToSleep := config.tryAddIMUReading(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.IMU.DataFrequencyHz())
	})
}

func onlineModeLidarTestHelper(
	ctx context.Context,
	t *testing.T,
	config Config,
	cf cartofacade.Mock,
	testLidar s.TestSensor,
) {
	logger := logging.NewTestLogger(t)
	dataFrequencyHz := 5

	lidar, err := s.NewLidar(context.Background(), s.SetupDeps(testLidar, s.NoMovementSensor), string(testLidar), dataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var calls []addLidarReadingArgs
	cf.AddLidarReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading s.TimedLidarSensorReadingResponse,
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

	config.CartoFacade = &cf
	config.Lidar = lidar
	config.Online = lidar.DataFrequencyHz() != 0

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
		test.That(t, call.sensorName, test.ShouldResemble, string(testLidar))
		// the lidar test fixture happens to always return the same pcd currently
		// in reality it could be a new pcd every time
		test.That(t, call.currentReading.Reading, test.ShouldResemble, expectedPCD)
		test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
	}

	if testLidar == s.GoodLidar {
		test.That(t, calls[0].currentReading.ReadingTime.Before(calls[1].currentReading.ReadingTime), test.ShouldBeTrue)
		test.That(t, calls[1].currentReading.ReadingTime.Before(calls[2].currentReading.ReadingTime), test.ShouldBeTrue)
	} else if testLidar == s.ReplayLidar {
		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTimestamp)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, calls[0].currentReading.ReadingTime.Equal(readingTime), test.ShouldBeTrue)
	} else {
		t.Errorf("no timestamp tests provided for %v", string(testLidar))
	}
}

func onlineModeIMUTestHelper(
	ctx context.Context,
	t *testing.T,
	config Config,
	cf cartofacade.Mock,
	testImu s.TestSensor,
) {
	logger := logging.NewTestLogger(t)
	dataFrequencyHz := 100
	imu, err := s.NewMovementSensor(context.Background(), s.SetupDeps(s.NoLidar, testImu), string(testImu), dataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var calls []addIMUReadingArgs
	cf.AddIMUReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading s.TimedIMUSensorReadingResponse,
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

	config.CartoFacade = &cf
	config.IMU = imu

	config.Online = true
	// config.currentLidarData.ReadingTime = time.Now().UTC().Add(-10 * time.Second)
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
		test.That(t, call.sensorName, test.ShouldResemble, string(testImu))
		// the IMU test fixture happens to always return the same readings currently
		// in reality they are likely different every time
		test.That(t, call.currentReading.LinearAcceleration, test.ShouldResemble, expectedIMUReading.LinearAcceleration)
		test.That(t, call.currentReading.AngularVelocity, test.ShouldResemble, expectedIMUReading.AngularVelocity)
		test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
	}

	if testImu == s.GoodIMU {
		test.That(t, calls[0].currentReading.ReadingTime.Before(calls[1].currentReading.ReadingTime), test.ShouldBeTrue)
		test.That(t, calls[1].currentReading.ReadingTime.Before(calls[2].currentReading.ReadingTime), test.ShouldBeTrue)
	} else if testImu == s.ReplayIMU {
		readingTime, err := time.Parse(time.RFC3339Nano, s.TestTimestamp)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, calls[0].currentReading.ReadingTime.Equal(readingTime), test.ShouldBeTrue)
	} else {
		t.Errorf("no timestamp tests provided for %v", string(testImu))
	}
}

func invalidLidarTestHelper(
	ctx context.Context,
	t *testing.T,
	cartoFacadeMock cartofacade.Mock,
	config Config,
	testLidar s.TestSensor,
	lidarDataFrequencyHz int,
) {
	logger := logging.NewTestLogger(t)
	lidar, err := s.NewLidar(context.Background(), s.SetupDeps(testLidar, s.NoMovementSensor), string(testLidar), lidarDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var calls []addLidarReadingArgs
	cartoFacadeMock.AddLidarReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading s.TimedLidarSensorReadingResponse,
	) error {
		args := addLidarReadingArgs{
			timeout:        timeout,
			sensorName:     sensorName,
			currentReading: currentReading,
		}
		calls = append(calls, args)
		return nil
	}
	config.Lidar = lidar

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
	testIMU s.TestSensor,
	imuDataFrequencyHz int,
) {
	logger := logging.NewTestLogger(t)
	imu, err := s.NewMovementSensor(context.Background(), s.SetupDeps(s.NoLidar, testIMU), string(testIMU), imuDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var calls []addIMUReadingArgs
	cartoFacadeMock.AddIMUReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading s.TimedIMUSensorReadingResponse,
	) error {
		args := addIMUReadingArgs{
			timeout:        timeout,
			sensorName:     sensorName,
			currentReading: currentReading,
		}
		calls = append(calls, args)
		return nil
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return lidarDataFrequencyHz }
	config.Lidar = &injectLidar

	config.IMU = imu

	jobDone := config.addIMUReading(ctx)
	test.That(t, len(calls), test.ShouldEqual, 0)
	test.That(t, jobDone, test.ShouldBeFalse)
}

func TestAddLidarReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	runFinalOptimizationFunc := func(context.Context, time.Duration) error {
		return nil
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	config := Config{
		Logger:                   logger,
		CartoFacade:              &cf,
		Lidar:                    &injectLidar,
		Timeout:                  10 * time.Second,
		RunFinalOptimizationFunc: runFinalOptimizationFunc,
		Mutex:                    &sync.Mutex{},
	}
	ctx := context.Background()

	t.Run("returns error in online mode when lidar GetData returns error, doesn't try to add lidar data", func(t *testing.T) {
		config.Online = true
		invalidLidarTestHelper(
			ctx,
			t,
			cf,
			config,
			s.LidarWithErroringFunctions,
			10,
		)
	})

	t.Run("returns error in online mode when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		config.Online = true
		invalidLidarTestHelper(
			ctx,
			t,
			cf,
			config,
			s.InvalidReplayLidar,
			10,
		)
	})

	t.Run("replay sensor adds sensor data until success in offline mode", func(t *testing.T) {
		config.Online = false
		lidar, imu := s.ReplayLidar, s.NoMovementSensor
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		var calls []addLidarReadingArgs
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarSensorReadingResponse,
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

		_ = config.addLidarReading(ctx) // first call gets data
		jobDone := config.addLidarReading(ctx)
		test.That(t, len(calls), test.ShouldEqual, 3)
		test.That(t, jobDone, test.ShouldBeFalse)

		firstTimestamp := calls[0].currentReading.ReadingTime
		for i, call := range calls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, string(lidar))
			test.That(t, call.currentReading.Reading, test.ShouldResemble, expectedPCD)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.currentReading.ReadingTime, test.ShouldEqual, firstTimestamp)
		}
	})

	t.Run("online replay lidar adds sensor reading once and ignores errors", func(t *testing.T) {
		config.Online = true
		onlineModeLidarTestHelper(ctx, t, config, cf, s.ReplayLidar)
	})

	t.Run("online lidar adds sensor reading once and ignores errors", func(t *testing.T) {
		config.Online = true
		onlineModeLidarTestHelper(ctx, t, config, cf, s.GoodLidar)
	})

	t.Run("returns true when lidar returns an error that it reached end of dataset and optimization function succeeds", func(t *testing.T) {
		config.Online = false
		lidar, imu := s.FinishedReplayLidar, s.NoMovementSensor
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor

		jobDone := config.addLidarReading(ctx)
		test.That(t, jobDone, test.ShouldBeTrue)
	})

	t.Run("returns true when lidar returns an error that it reached end of dataset and optimization function fails", func(t *testing.T) {
		config.Online = false
		runFinalOptimizationFunc = func(context.Context, time.Duration) error {
			return errors.New("test error")
		}
		config.RunFinalOptimizationFunc = runFinalOptimizationFunc

		lidar, imu := s.FinishedReplayLidar, s.NoMovementSensor
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor

		jobDone := config.addLidarReading(ctx)
		test.That(t, jobDone, test.ShouldBeTrue)
	})
}

func TestAddIMUReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	cf := cartofacade.Mock{}

	injectImu := inject.TimedMovementSensor{}
	injectImu.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IMU:         &injectImu,
		Timeout:     10 * time.Second,
		Mutex:       &sync.Mutex{},
	}

	t.Run("returns error in online mode when IMU GetData returns error, doesn't try to add IMU data", func(t *testing.T) {
		invalidIMUTestHelper(
			ctx,
			t,
			cf,
			config,
			10,
			s.IMUWithErroringFunctions,
			10,
		)
	})

	t.Run("returns error in online mode when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		invalidIMUTestHelper(
			ctx,
			t,
			cf,
			config,
			10,
			s.InvalidReplayIMU,
			10,
		)
	})

	t.Run("replay sensor adds IMU data until success in offline mode", func(t *testing.T) {
		lidar, imu := s.NoLidar, s.ReplayIMU
		dataFrequencyHz := 0
		replayIMU, err := s.NewMovementSensor(context.Background(), s.SetupDeps(lidar, imu), string(imu), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		injectLidar := inject.TimedLidar{}
		injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

		var calls []addIMUReadingArgs
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUSensorReadingResponse,
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
		config.IMU = replayIMU
		config.Lidar = &injectLidar
		config.currentLidarData = &s.TimedLidarSensorReadingResponse{
			ReadingTime: time.Now().UTC().Add(-10 * time.Second),
		}
		config.sensorProcessStartTime = time.Time{}.Add(time.Millisecond)

		_ = config.addIMUReading(ctx) // first call gets data
		jobDone := config.addIMUReading(ctx)
		test.That(t, len(calls), test.ShouldEqual, 3)
		test.That(t, jobDone, test.ShouldBeFalse)

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

	t.Run("online replay IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeIMUTestHelper(ctx, t, config, cf, s.ReplayIMU)
	})

	t.Run("online IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeIMUTestHelper(ctx, t, config, cf, s.GoodIMU)
	})

	t.Run("returns true when IMU returns an error that it reached end of dataset", func(t *testing.T) {
		lidar, imu := s.NoLidar, s.FinishedReplayIMU
		dataFrequencyHz := 0
		replayIMU, err := s.NewMovementSensor(context.Background(), s.SetupDeps(lidar, imu), string(imu), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.IMU = replayIMU
		config.currentLidarData = &s.TimedLidarSensorReadingResponse{
			ReadingTime: time.Now().UTC().Add(-10 * time.Second),
		}

		jobDone := config.addIMUReading(ctx)
		test.That(t, jobDone, test.ShouldBeTrue)
	})
}

func TestStartLidar(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	config := Config{
		Logger:                   logger,
		CartoFacade:              &cf,
		Lidar:                    &injectLidar,
		Timeout:                  10 * time.Second,
		RunFinalOptimizationFunc: cf.RunFinalOptimization,
		Mutex:                    &sync.Mutex{},
	}
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	t.Run("returns true when lidar returns an error that it reached end of dataset but the context is valid", func(t *testing.T) {
		lidar, imu := s.FinishedReplayLidar, s.NoMovementSensor
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor

		jobDone := config.StartLidar(context.Background())
		test.That(t, jobDone, test.ShouldBeTrue)
	})

	t.Run("returns false when lidar returns an error that it reached end of dataset but the context was cancelled", func(t *testing.T) {
		lidar, imu := s.FinishedReplayLidar, s.NoMovementSensor
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor

		cancelFunc()

		jobDone := config.StartLidar(cancelCtx)
		test.That(t, jobDone, test.ShouldBeFalse)
	})
}

func TestStartIMU(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	injectImu := inject.TimedMovementSensor{}
	injectImu.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IMU:         &injectImu,
		Timeout:     10 * time.Second,
		Mutex:       &sync.Mutex{},
	}
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	t.Run("returns true when IMU returns an error that it reached end of dataset but the context is valid", func(t *testing.T) {
		lidar, imu := s.NoLidar, s.FinishedReplayIMU
		dataFrequencyHz := 0
		replaySensor, err := s.NewMovementSensor(context.Background(), s.SetupDeps(lidar, imu), string(imu), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		injectLidar := inject.TimedLidar{}
		injectLidar.DataFrequencyHzFunc = func() int { return 0 }

		config.IMU = replaySensor
		config.Lidar = &injectLidar
		config.currentLidarData = &s.TimedLidarSensorReadingResponse{
			ReadingTime: time.Now().UTC().Add(-10 * time.Second),
		}

		jobDone := config.StartIMU(context.Background())
		test.That(t, jobDone, test.ShouldBeTrue)
	})

	t.Run("returns false when IMU returns an error that it reached end of dataset but the context was cancelled", func(t *testing.T) {
		lidar, imu := s.NoLidar, s.FinishedReplayIMU
		dataFrequencyHz := 0
		replaySensor, err := s.NewMovementSensor(context.Background(), s.SetupDeps(lidar, imu), string(imu), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.IMU = replaySensor
		config.currentLidarData = &s.TimedLidarSensorReadingResponse{
			ReadingTime: time.Now().UTC().Add(-10 * time.Second),
		}

		cancelFunc()

		jobDone := config.StartLidar(cancelCtx)
		test.That(t, jobDone, test.ShouldBeFalse)
	})
}
