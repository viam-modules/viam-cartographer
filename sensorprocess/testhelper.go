package sensorprocess

import (
	"context"
	"errors"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/inject"
	rdkutils "go.viam.com/rdk/utils"
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

type addOdometerReadingArgs struct {
	timeout        time.Duration
	sensorName     string
	currentReading s.TimedOdometerReadingResponse
}

var (
	//nolint:dupword
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

func validAddLidarReadingInOnlineTestHelper(
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

func invalidAddLidarReadingInOnlineTestHelper(
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
	config.CartoFacade = &cartoFacadeMock

	err = config.addLidarReadingInOnline(ctx)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, len(calls), test.ShouldEqual, 0)
}

func validAddMovementSensorReadingInOnlineTestHelper(
	ctx context.Context,
	t *testing.T,
	config Config,
	cf cartofacade.Mock,
	testMovementSensor s.TestSensor,
) ([]addIMUReadingArgs, []addOdometerReadingArgs) {
	logger := logging.NewTestLogger(t)
	dataFrequencyHz := 100
	movementSensor, err := s.NewMovementSensor(context.Background(), s.SetupDeps(s.NoLidar, testMovementSensor),
		string(testMovementSensor), dataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var imuCalls []addIMUReadingArgs
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
		imuCalls = append(imuCalls, args)
		if len(imuCalls) == 1 {
			return errUnknown
		}
		if len(imuCalls) == 2 {
			return cartofacade.ErrUnableToAcquireLock
		}
		return nil
	}

	var odometerCalls []addOdometerReadingArgs
	cf.AddOdometerReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading s.TimedOdometerReadingResponse,
	) error {
		args := addOdometerReadingArgs{
			timeout:        timeout,
			sensorName:     sensorName,
			currentReading: currentReading,
		}
		odometerCalls = append(odometerCalls, args)
		if len(odometerCalls) == 1 {
			return errUnknown
		}
		if len(odometerCalls) == 2 {
			return cartofacade.ErrUnableToAcquireLock
		}
		return nil
	}

	config.CartoFacade = &cf
	config.MovementSensor = movementSensor
	config.IsOnline = true

	testNumberCalls := func(movementSensor s.TimedMovementSensor, expectedNumberCalls int) {
		if movementSensor.Properties().IMUSupported {
			test.That(t, len(imuCalls), test.ShouldEqual, expectedNumberCalls)
		} else {
			test.That(t, len(imuCalls), test.ShouldEqual, 0)
		}
		if movementSensor.Properties().OdometerSupported {
			test.That(t, len(odometerCalls), test.ShouldEqual, expectedNumberCalls)
		} else {
			test.That(t, len(odometerCalls), test.ShouldEqual, 0)
		}
	}

	err = config.addMovementSensorReadingInOnline(ctx)
	test.That(t, err, test.ShouldBeNil)
	testNumberCalls(movementSensor, 1)

	err = config.addMovementSensorReadingInOnline(ctx)
	test.That(t, err, test.ShouldBeNil)
	testNumberCalls(movementSensor, 2)

	err = config.addMovementSensorReadingInOnline(ctx)
	test.That(t, err, test.ShouldBeNil)
	testNumberCalls(movementSensor, 3)

	if movementSensor.Properties().IMUSupported {
		for i, call := range imuCalls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, string(testMovementSensor))
			// the IMU test fixture happens to always return the same readings currently
			// in reality they are likely different every time
			test.That(t, call.currentReading.LinearAcceleration, test.ShouldResemble, s.TestLinAcc)
			test.That(t, call.currentReading.AngularVelocity, test.ShouldResemble, spatialmath.AngularVelocity{
				X: rdkutils.DegToRad(s.TestAngVel.X),
				Y: rdkutils.DegToRad(s.TestAngVel.Y),
				Z: rdkutils.DegToRad(s.TestAngVel.Z),
			})
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
		}
	}

	if movementSensor.Properties().OdometerSupported {
		for i, call := range odometerCalls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, string(testMovementSensor))
			// the odometer test fixture happens to always return the same readings currently
			// in reality they are likely different every time
			test.That(t, call.currentReading.Position, test.ShouldResemble, s.TestPosition)
			test.That(t, call.currentReading.Orientation, test.ShouldResemble, s.TestOrientation)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
		}
	}

	return imuCalls, odometerCalls
}

func invalidAddMovementSensorReadingInOnlineTestHelper(
	ctx context.Context,
	t *testing.T,
	cartoFacadeMock cartofacade.Mock,
	config Config,
	lidarDataFrequencyHz int,
	testMovementSensor s.TestSensor,
	movementSensorDataFrequencyHz int,
) {
	logger := logging.NewTestLogger(t)
	movementSensor, err := s.NewMovementSensor(context.Background(), s.SetupDeps(s.NoLidar, testMovementSensor),
		string(testMovementSensor), movementSensorDataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var imuCalls []addIMUReadingArgs
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
		imuCalls = append(imuCalls, args)
		return nil
	}

	var odometerCalls []addOdometerReadingArgs
	cartoFacadeMock.AddOdometerReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading s.TimedOdometerReadingResponse,
	) error {
		args := addOdometerReadingArgs{
			timeout:        timeout,
			sensorName:     sensorName,
			currentReading: currentReading,
		}
		odometerCalls = append(odometerCalls, args)
		return nil
	}
	config.CartoFacade = &cartoFacadeMock

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return lidarDataFrequencyHz }
	config.Lidar = &injectLidar

	config.MovementSensor = movementSensor

	err = config.addMovementSensorReadingInOnline(ctx)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, len(imuCalls), test.ShouldEqual, 0)
	test.That(t, len(odometerCalls), test.ShouldEqual, 0)
}

func validAddMovementSensorReadingUntilSuccessTestHelper(
	ctx context.Context,
	t *testing.T,
	config Config,
	cf cartofacade.Mock,
	testMovementSensor s.TestSensor,
) {
	logger := logging.NewTestLogger(t)
	dataFrequencyHz := 0
	movementSensor, err := s.NewMovementSensor(context.Background(), s.SetupDeps(s.NoLidar, testMovementSensor),
		string(testMovementSensor), dataFrequencyHz, logger)
	test.That(t, err, test.ShouldBeNil)

	var imuCalls []addIMUReadingArgs
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
		imuCalls = append(imuCalls, args)
		if len(imuCalls) == 1 {
			return errUnknown
		}
		if len(imuCalls) == 2 {
			return cartofacade.ErrUnableToAcquireLock
		}
		return nil
	}

	var odometerCalls []addOdometerReadingArgs
	cf.AddOdometerReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading s.TimedOdometerReadingResponse,
	) error {
		args := addOdometerReadingArgs{
			timeout:        timeout,
			sensorName:     sensorName,
			currentReading: currentReading,
		}
		odometerCalls = append(odometerCalls, args)
		if len(odometerCalls) == 1 {
			return errUnknown
		}
		if len(odometerCalls) == 2 {
			return cartofacade.ErrUnableToAcquireLock
		}
		return nil
	}

	config.CartoFacade = &cf
	config.MovementSensor = movementSensor
	config.IsOnline = true

	now := time.Now()
	var movementSensorReading s.TimedMovementSensorReadingResponse
	if movementSensor.Properties().IMUSupported {
		movementSensorReading.TimedIMUResponse = &s.TimedIMUReadingResponse{
			AngularVelocity:    s.TestAngVel,
			LinearAcceleration: s.TestLinAcc,
			ReadingTime:        now,
		}
	}
	if movementSensor.Properties().OdometerSupported {
		movementSensorReading.TimedOdometerResponse = &s.TimedOdometerReadingResponse{
			Position:    s.TestPosition,
			Orientation: s.TestOrientation,
			ReadingTime: now,
		}
	}

	config.tryAddMovementSensorReadingUntilSuccess(ctx, movementSensorReading)
	if movementSensor.Properties().IMUSupported {
		test.That(t, len(imuCalls), test.ShouldEqual, 3)
		firstTimestamp := imuCalls[0].currentReading.ReadingTime
		for i, call := range imuCalls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, string(testMovementSensor))
			test.That(t, call.currentReading.LinearAcceleration, test.ShouldResemble, movementSensorReading.TimedIMUResponse.LinearAcceleration)
			test.That(t, call.currentReading.AngularVelocity, test.ShouldResemble, movementSensorReading.TimedIMUResponse.AngularVelocity)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.currentReading.ReadingTime, test.ShouldEqual, firstTimestamp)
		}
	}
	if movementSensor.Properties().OdometerSupported {
		test.That(t, len(odometerCalls), test.ShouldEqual, 3)
		firstTimestamp := odometerCalls[0].currentReading.ReadingTime
		for i, call := range odometerCalls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, string(testMovementSensor))
			test.That(t, call.currentReading.Position, test.ShouldResemble, movementSensorReading.TimedOdometerResponse.Position)
			test.That(t, call.currentReading.Orientation, test.ShouldResemble, movementSensorReading.TimedOdometerResponse.Orientation)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.currentReading.ReadingTime, test.ShouldEqual, firstTimestamp)
		}
	}
}

func TestTryAddMovementSensorReadingOnceTestHelper(
	ctx context.Context,
	t *testing.T,
	config Config,
	cf cartofacade.Mock,
) {
	config.CartoFacade = &cf

	// Set up movement sensor reading
	now := time.Now().UTC()
	var movementSensorReading s.TimedMovementSensorReadingResponse
	if config.MovementSensor.Properties().IMUSupported {
		movementSensorReading.TimedIMUResponse = &s.TimedIMUReadingResponse{
			AngularVelocity:    s.TestAngVel,
			LinearAcceleration: s.TestLinAcc,
			ReadingTime:        now,
		}
	}
	if config.MovementSensor.Properties().OdometerSupported {
		movementSensorReading.TimedOdometerResponse = &s.TimedOdometerReadingResponse{
			Position:    s.TestPosition,
			Orientation: s.TestOrientation,
			ReadingTime: now,
		}
	}

	if config.MovementSensor.Properties().IMUSupported {

		// In case that the odometer is also supported, let's assume it works fast and efficiently for all the
		// following test cases
		cf.AddOdometerReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedOdometerReadingResponse,
		) error {
			return nil
		}

		t.Run("when AddIMUReading blocks for more than the date rate and succeeds, time to sleep is 0", func(t *testing.T) {
			var imuCalls []addIMUReadingArgs
			cf.AddIMUReadingFunc = func(
				ctx context.Context,
				timeout time.Duration,
				sensorName string,
				currentReading s.TimedIMUReadingResponse,
			) error {
				time.Sleep(1 * time.Second)
				args := addIMUReadingArgs{
					timeout:        timeout,
					sensorName:     sensorName,
					currentReading: currentReading,
				}
				imuCalls = append(imuCalls, args)
				return nil
			}

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
			test.That(t, timeToSleep, test.ShouldEqual, 0)

			if config.MovementSensor.Properties().IMUSupported {
				test.That(t, len(imuCalls), test.ShouldEqual, 1)
				for i, call := range imuCalls {
					t.Logf("call %d", i)
					test.That(t, call.sensorName, test.ShouldResemble, config.MovementSensor.Name())
					test.That(t, call.currentReading.LinearAcceleration, test.ShouldResemble, movementSensorReading.TimedIMUResponse.LinearAcceleration)
					test.That(t, call.currentReading.AngularVelocity, test.ShouldResemble, movementSensorReading.TimedIMUResponse.AngularVelocity)
					test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
				}
			}
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

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
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

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
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

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
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

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
			test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
			test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
		})

		t.Run("when AddIMUReading or AddOdometerReading are faster than date rate "+
			"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
			cf.AddIMUReadingFunc = func(
				ctx context.Context,
				timeout time.Duration,
				sensorName string,
				currentReading s.TimedIMUReadingResponse,
			) error {
				return errUnknown
			}

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
			test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
			test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
		})

	}

	if config.MovementSensor.Properties().OdometerSupported {

		// In case that the IMU is also supported, let's assume it works fast and efficiently for all the
		// following test cases
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			return errUnknown
		}

		t.Run("when AddOdometerReading blocks for more than the date rate and succeeds, time to sleep is 0", func(t *testing.T) {
			var odometerCalls []addOdometerReadingArgs
			cf.AddOdometerReadingFunc = func(
				ctx context.Context,
				timeout time.Duration,
				sensorName string,
				currentReading s.TimedOdometerReadingResponse,
			) error {
				time.Sleep(1 * time.Second)
				args := addOdometerReadingArgs{
					timeout:        timeout,
					sensorName:     sensorName,
					currentReading: currentReading,
				}
				odometerCalls = append(odometerCalls, args)
				return nil
			}

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
			test.That(t, timeToSleep, test.ShouldEqual, 0)

			if config.MovementSensor.Properties().OdometerSupported {
				test.That(t, len(odometerCalls), test.ShouldEqual, 1)
				firstTimestamp := odometerCalls[0].currentReading.ReadingTime
				for i, call := range odometerCalls {
					t.Logf("call %d", i)
					test.That(t, call.sensorName, test.ShouldResemble, config.MovementSensor.Name())
					test.That(t, call.currentReading.Position, test.ShouldResemble, movementSensorReading.TimedOdometerResponse.Position)
					test.That(t, call.currentReading.Orientation, test.ShouldResemble, movementSensorReading.TimedOdometerResponse.Orientation)
					test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
					test.That(t, call.currentReading.ReadingTime, test.ShouldEqual, firstTimestamp)
				}
			}
		})

		t.Run("when AddOdometerReading is slower than date rate and returns a lock error, time to sleep is 0", func(t *testing.T) {
			cf.AddOdometerReadingFunc = func(
				ctx context.Context,
				timeout time.Duration,
				sensorName string,
				currentReading s.TimedOdometerReadingResponse,
			) error {
				time.Sleep(1 * time.Second)
				return cartofacade.ErrUnableToAcquireLock
			}

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
			test.That(t, timeToSleep, test.ShouldEqual, 0)
		})

		t.Run("when AddOdometerReading blocks for more than the date rate and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
			cf.AddOdometerReadingFunc = func(
				ctx context.Context,
				timeout time.Duration,
				sensorName string,
				currentReading s.TimedOdometerReadingResponse,
			) error {
				time.Sleep(1 * time.Second)
				return errUnknown
			}

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
			test.That(t, timeToSleep, test.ShouldEqual, 0)
		})

		t.Run("when AddOdometerReading are faster than the date rate and succeeds, time to sleep is <= date rate", func(t *testing.T) {
			cf.AddOdometerReadingFunc = func(
				ctx context.Context,
				timeout time.Duration,
				sensorName string,
				currentReading s.TimedOdometerReadingResponse,
			) error {
				return nil
			}

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
			test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
			test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
		})

		t.Run("when AddOdometerReading are faster than the date rate and returns a lock error, time to sleep is <= date rate", func(t *testing.T) {
			cf.AddOdometerReadingFunc = func(
				ctx context.Context,
				timeout time.Duration,
				sensorName string,
				currentReading s.TimedOdometerReadingResponse,
			) error {
				return cartofacade.ErrUnableToAcquireLock
			}

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
			test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
			test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
		})

		t.Run("when AddOdometerReading are faster than date rate "+
			"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
			cf.AddOdometerReadingFunc = func(
				ctx context.Context,
				timeout time.Duration,
				sensorName string,
				currentReading s.TimedOdometerReadingResponse,
			) error {
				return errUnknown
			}

			timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), movementSensorReading)
			test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
			test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
		})

	}
}
