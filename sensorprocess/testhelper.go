package sensorprocess

import (
	"context"
	"errors"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/inject"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

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

	expectedIMUReading = s.TimedIMUReadingResponse{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 0.017453292519943295, Y: 0.008726646259971648, Z: 0},
	}
	errUnknown = errors.New("unknown error")
)

type addLidarReadingArgs struct {
	timeout        time.Duration
	sensorName     string
	currentReading s.TimedLidarReadingResponse
}

type addIMUReadingArgs struct {
	timeout        time.Duration
	sensorName     string
	currentReading s.TimedIMUReadingResponse
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

	config.CartoFacade = &cf
	config.Lidar = lidar
	config.IsOnline = lidar.DataFrequencyHz() != 0

	err = config.addLidarReadingInOnline(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(calls), test.ShouldEqual, 1)

	err = config.addLidarReadingInOnline(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(calls), test.ShouldEqual, 2)

	err = config.addLidarReadingInOnline(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(calls), test.ShouldEqual, 3)

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

	config.CartoFacade = &cf
	config.IMU = imu
	config.IsOnline = true

	err = config.addIMUReadingInOnline(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(calls), test.ShouldEqual, 1)

	err = config.addIMUReadingInOnline(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(calls), test.ShouldEqual, 2)

	err = config.addIMUReadingInOnline(ctx)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(calls), test.ShouldEqual, 3)

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

func invalidOnlineModeLidarTestHelper(
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
		currentReading s.TimedLidarReadingResponse,
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

	err = config.addLidarReadingInOnline(ctx)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, len(calls), test.ShouldEqual, 0)
}

func invalidOnlineModeIMUTestHelper(
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
		currentReading s.TimedIMUReadingResponse,
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

	err = config.addIMUReadingInOnline(ctx)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, len(calls), test.ShouldEqual, 0)
}
