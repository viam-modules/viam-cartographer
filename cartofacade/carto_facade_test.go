//nolint:dupl
package cartofacade

import (
	"bytes"
	"context"
	"errors"
	"os"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.uber.org/multierr"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"
)

const timeoutErrMessage = "timeout reading from cartographer"

func TestRequest(t *testing.T) {
	cartoLib := CartoLibMock{}
	cartoLib.TerminateFunc = func() error {
		return nil
	}
	testErr := errors.New("error")
	t.Run("successful request", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := GetTestConfig("mysensor", "")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := GetTestAlgoConfig(false)
		carto := CartoMock{}
		carto.PositionFunc = func() (Position, error) {
			return Position{}, nil
		}

		cf := New(&cartoLib, config, algoConfig)
		cf.carto = &carto
		cf.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

		res, err := cf.request(cancelCtx, position, map[RequestParamType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, res, test.ShouldNotBeNil)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("failed request", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := GetTestConfig("mysensor", "")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := GetTestAlgoConfig(false)
		carto := CartoMock{}

		carto.StartFunc = func() error {
			return testErr
		}

		cf := New(&cartoLib, config, algoConfig)
		cf.carto = &carto
		cf.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

		_, err = cf.request(cancelCtx, start, map[RequestParamType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeError)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("request with a cancelled context", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := GetTestConfig("mysensor", "")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := GetTestAlgoConfig(false)
		carto := CartoMock{}

		carto.StartFunc = func() error {
			return testErr
		}

		cf := New(&cartoLib, config, algoConfig)
		cf.carto = &carto
		cf.startCGoroutine(cancelCtx, &activeBackgroundWorkers)
		cancelFunc()
		activeBackgroundWorkers.Wait()

		_, err = cf.request(cancelCtx, start, map[RequestParamType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		errMsg := "timeout writing to cartographer"
		expectedErr := multierr.Combine(errors.New(errMsg), context.Canceled)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("request with a work function that takes longer than the timeout", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := GetTestConfig("mysensor", "")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := GetTestAlgoConfig(false)
		carto := CartoMock{}

		carto.StartFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}

		cf := New(&cartoLib, config, algoConfig)
		cf.carto = &carto
		cf.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

		_, err = cf.request(cancelCtx, start, map[RequestParamType]interface{}{}, 10*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	err := cartoLib.Terminate()
	test.That(t, err, test.ShouldBeNil)
}

func TestInitialize(t *testing.T) {
	lib, err := NewLib(1, 1)
	test.That(t, err, test.ShouldBeNil)

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor", "")
	algoCfg := GetTestAlgoConfig(false)
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)

	t.Run("success", func(t *testing.T) {
		slamMode, err := cartoFacade.Initialize(cancelCtx, 5*time.Second, &activeBackgroundWorkers)
		test.That(t, slamMode, test.ShouldEqual, MappingMode)
		test.That(t, err, test.ShouldBeNil)
	})

	err = cartoFacade.Terminate(cancelCtx, 5*time.Second)
	test.That(t, err, test.ShouldBeNil)
	err = lib.Terminate()
	test.That(t, err, test.ShouldBeNil)
	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestStart(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor", "")
	algoCfg := GetTestAlgoConfig(false)
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	cartoFacade.carto = &carto
	cartoFacade.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

	t.Run("success", func(t *testing.T) {
		carto.StartFunc = func() error {
			return nil
		}
		cartoFacade.carto = &carto
		err = cartoFacade.Start(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("failure", func(t *testing.T) {
		expectedErr := errors.New("Start failed")
		carto.StartFunc = func() error {
			return expectedErr
		}
		cartoFacade.carto = &carto
		err = cartoFacade.Start(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("times out", func(t *testing.T) {
		carto.StartFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto
		err = cartoFacade.Start(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestStop(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor", "")
	algoCfg := GetTestAlgoConfig(false)
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	cartoFacade.carto = &carto
	cartoFacade.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

	t.Run("success", func(t *testing.T) {
		carto.StopFunc = func() error {
			return nil
		}
		err = cartoFacade.Stop(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("failure", func(t *testing.T) {
		expectedErr := errors.New("Stop failed")
		carto.StopFunc = func() error {
			return expectedErr
		}
		cartoFacade.carto = &carto
		err = cartoFacade.Stop(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("times out", func(t *testing.T) {
		carto.StopFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto
		err = cartoFacade.Stop(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestTerminate(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor", "")
	algoCfg := GetTestAlgoConfig(false)
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	cartoFacade.carto = &carto
	cartoFacade.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

	t.Run("success", func(t *testing.T) {
		carto.TerminateFunc = func() error {
			return nil
		}
		err = cartoFacade.Terminate(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("failure", func(t *testing.T) {
		expectedErr := errors.New("Terminate failed")
		carto.TerminateFunc = func() error {
			return expectedErr
		}
		cartoFacade.carto = &carto
		err = cartoFacade.Terminate(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("times out", func(t *testing.T) {
		carto.TerminateFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto
		err = cartoFacade.Terminate(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestAddLidarReading(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor", "")
	test.That(t, err, test.ShouldBeNil)
	algoCfg := GetTestAlgoConfig(false)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	cartoFacade.carto = &carto
	cartoFacade.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

	timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)

	// read PCD
	file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/0.pcd"))
	test.That(t, err, test.ShouldBeNil)
	buf := new(bytes.Buffer)
	pc, err := pointcloud.ReadPCD(file)
	test.That(t, err, test.ShouldBeNil)
	err = pointcloud.ToPCD(pc, buf, 0)
	test.That(t, err, test.ShouldBeNil)

	t.Run("success", func(t *testing.T) {
		carto.AddLidarReadingFunc = func(name string, reading []byte, time time.Time) error {
			return nil
		}
		err = cartoFacade.AddLidarReading(cancelCtx, 5*time.Second, "mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("failure", func(t *testing.T) {
		expectedErr := errors.New("AddLidarReading failed")
		carto.AddLidarReadingFunc = func(name string, reading []byte, time time.Time) error {
			return expectedErr
		}
		cartoFacade.carto = &carto
		err = cartoFacade.AddLidarReading(cancelCtx, 5*time.Second, "mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("times out", func(t *testing.T) {
		carto.AddLidarReadingFunc = func(name string, reading []byte, timestamp time.Time) error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto
		err = cartoFacade.AddLidarReading(cancelCtx, 1*time.Millisecond, "mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestAddIMUReading(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mylidar", "myIMU")
	test.That(t, err, test.ShouldBeNil)
	algoCfg := GetTestAlgoConfig(true)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	cartoFacade.carto = &carto
	cartoFacade.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

	timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
	testIMUReading := IMUReading{
		LinearAcceleration: r3.Vector{X: 0.1, Y: 0, Z: 9.8},
		AngularVelocity:    spatialmath.AngularVelocity{X: 0, Y: -0.2, Z: 0},
	}

	t.Run("success", func(t *testing.T) {
		carto.AddIMUReadingFunc = func(name string, reading IMUReading, time time.Time) error {
			return nil
		}
		err = cartoFacade.AddIMUReading(cancelCtx, 5*time.Second, "myIMU", testIMUReading, timestamp)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("failure", func(t *testing.T) {
		expectedErr := errors.New("AddIMUReading failed")
		carto.AddIMUReadingFunc = func(name string, reading IMUReading, time time.Time) error {
			return expectedErr
		}
		cartoFacade.carto = &carto

		// returns error
		err = cartoFacade.AddIMUReading(cancelCtx, 5*time.Second, "myIMU", testIMUReading, timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("times out", func(t *testing.T) {
		carto.AddIMUReadingFunc = func(name string, reading IMUReading, timestamp time.Time) error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto
		err = cartoFacade.AddIMUReading(cancelCtx, 1*time.Millisecond, "myIMU", testIMUReading, timestamp)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestPosition(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor", "")
	algoCfg := GetTestAlgoConfig(false)
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	cartoFacade.carto = &carto
	cartoFacade.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

	t.Run("success", func(t *testing.T) {
		carto.PositionFunc = func() (Position, error) {
			pos := Position{X: 1, Y: 2, Z: 3}
			return pos, nil
		}
		cartoFacade.carto = &carto
		pos, err := cartoFacade.Position(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pos.X, test.ShouldEqual, 1)
		test.That(t, pos.Y, test.ShouldEqual, 2)
		test.That(t, pos.Z, test.ShouldEqual, 3)
	})

	t.Run("failure", func(t *testing.T) {
		expectedErr := errors.New("Position failed")
		carto.PositionFunc = func() (Position, error) {
			return Position{}, expectedErr
		}
		cartoFacade.carto = &carto
		_, err = cartoFacade.Position(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("times out", func(t *testing.T) {
		carto.PositionFunc = func() (Position, error) {
			time.Sleep(50 * time.Millisecond)
			return Position{}, nil
		}
		cartoFacade.carto = &carto
		_, err = cartoFacade.Position(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestInternalState(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor", "")
	algoCfg := GetTestAlgoConfig(false)
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	cartoFacade.carto = &carto
	cartoFacade.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

	t.Run("success", func(t *testing.T) {
		internalState := []byte("hello!")
		carto.InternalStateFunc = func() ([]byte, error) {
			return internalState, nil
		}
		cartoFacade.carto = &carto
		internalState, err := cartoFacade.InternalState(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldResemble, internalState)
	})

	t.Run("failure", func(t *testing.T) {
		expectedErr := errors.New("InternalState failed")
		carto.InternalStateFunc = func() ([]byte, error) {
			return []byte{}, expectedErr
		}
		cartoFacade.carto = &carto
		_, err = cartoFacade.InternalState(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("times out", func(t *testing.T) {
		carto.InternalStateFunc = func() ([]byte, error) {
			time.Sleep(50 * time.Millisecond)
			return []byte{}, nil
		}
		cartoFacade.carto = &carto
		_, err = cartoFacade.InternalState(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestPointCloudMap(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor", "")
	algoCfg := GetTestAlgoConfig(false)
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	cartoFacade.carto = &carto
	cartoFacade.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

	t.Run("success", func(t *testing.T) {
		internalState := []byte("hello!")
		carto.PointCloudMapFunc = func() ([]byte, error) {
			return internalState, nil
		}
		cartoFacade.carto = &carto
		internalState, err := cartoFacade.PointCloudMap(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldResemble, internalState)
	})

	t.Run("failure", func(t *testing.T) {
		expectedErr := errors.New("PointCloudMap failed")
		carto.PointCloudMapFunc = func() ([]byte, error) {
			return []byte{}, expectedErr
		}
		cartoFacade.carto = &carto
		_, err = cartoFacade.PointCloudMap(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("times out", func(t *testing.T) {
		carto.PointCloudMapFunc = func() ([]byte, error) {
			time.Sleep(50 * time.Millisecond)
			return []byte{}, nil
		}
		cartoFacade.carto = &carto
		_, err = cartoFacade.PointCloudMap(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestRunFinalOptimization(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor", "")
	algoCfg := GetTestAlgoConfig(false)
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	cartoFacade.carto = &carto
	cartoFacade.startCGoroutine(cancelCtx, &activeBackgroundWorkers)

	t.Run("success", func(t *testing.T) {
		carto.RunFinalOptimizationFunc = func() error {
			return nil
		}
		err = cartoFacade.RunFinalOptimization(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("failure", func(t *testing.T) {
		expectedErr := errors.New("RunFinalOptimization failed")
		carto.RunFinalOptimizationFunc = func() error {
			return expectedErr
		}
		cartoFacade.carto = &carto
		err = cartoFacade.RunFinalOptimization(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("times out", func(t *testing.T) {
		carto.RunFinalOptimizationFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto
		err = cartoFacade.RunFinalOptimization(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := multierr.Combine(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}
