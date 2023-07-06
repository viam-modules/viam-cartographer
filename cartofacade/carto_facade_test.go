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

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"
)

const timeoutErrMessage = "timeout has occurred while trying to read request from cartofacade"

func TestRequest(t *testing.T) {
	cartoLib := CartoLibMock{}
	cartoLib.TerminateFunc = func() error {
		return nil
	}
	testErr := errors.New("error")
	t.Run("test successful request", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := GetTestAlgoConfig()
		carto := CartoMock{}
		carto.GetPositionFunc = func() (GetPosition, error) {
			return GetPosition{}, nil
		}

		cf := New(&cartoLib, config, algoConfig)
		cf.carto = &carto
		cf.start(cancelCtx, &activeBackgroundWorkers)

		res, err := cf.request(cancelCtx, position, map[RequestParamType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, res, test.ShouldNotBeNil)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("test failed request", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := GetTestAlgoConfig()
		carto := CartoMock{}

		carto.StartFunc = func() error {
			return testErr
		}

		cf := New(&cartoLib, config, algoConfig)
		cf.carto = &carto
		cf.start(cancelCtx, &activeBackgroundWorkers)

		_, err = cf.request(cancelCtx, start, map[RequestParamType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeError)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("test requesting with a canceled context", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := GetTestAlgoConfig()
		carto := CartoMock{}

		carto.StartFunc = func() error {
			return testErr
		}

		cf := New(&cartoLib, config, algoConfig)
		cf.carto = &carto
		cf.start(cancelCtx, &activeBackgroundWorkers)
		cancelFunc()
		activeBackgroundWorkers.Wait()

		_, err = cf.request(cancelCtx, start, map[RequestParamType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		errMsg := "timeout has occurred while trying to write request to cartofacade. Did you forget to call Start()?"
		expectedErr := errors.Join(errors.New(errMsg), context.Canceled)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	t.Run("test requesting with when the work function takes longer than the timeout", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := GetTestAlgoConfig()
		carto := CartoMock{}

		carto.StartFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}

		cf := New(&cartoLib, config, algoConfig)
		cf.carto = &carto
		cf.start(cancelCtx, &activeBackgroundWorkers)

		_, err = cf.request(cancelCtx, start, map[RequestParamType]interface{}{}, 10*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := errors.Join(errors.New(timeoutErrMessage), context.DeadlineExceeded)
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

	cfg, dir, err := GetTestConfig("mysensor")
	algoCfg := GetTestAlgoConfig()
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)

	t.Run("test Initialize - successful case", func(t *testing.T) {
		err = cartoFacade.Initialize(cancelCtx, 5*time.Second, &activeBackgroundWorkers)
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

	cfg, dir, err := GetTestConfig("mysensor")
	algoCfg := GetTestAlgoConfig()
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	carto.StartFunc = func() error {
		return nil
	}
	cartoFacade.carto = &carto
	cartoFacade.Initialize(cancelCtx, 5*time.Second, &activeBackgroundWorkers)
	defer cartoFacade.Terminate(cancelCtx, 5*time.Second)

	t.Run("testing Start", func(t *testing.T) {
		// success case
		err = cartoFacade.Start(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)

		carto.StartFunc = func() error {
			return errors.New("test error 1")
		}
		cartoFacade.carto = &carto

		// returns error
		err = cartoFacade.Start(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("test error 1"))

		carto.StartFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto

		// times out
		err = cartoFacade.Start(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := errors.Join(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestStop(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor")
	algoCfg := GetTestAlgoConfig()
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	carto.StopFunc = func() error {
		return nil
	}
	cartoFacade.carto = &carto
	cartoFacade.Initialize(cancelCtx, 5*time.Second, &activeBackgroundWorkers)
	defer cartoFacade.Terminate(cancelCtx, 5*time.Second)

	t.Run("testing Stop", func(t *testing.T) {
		// success case
		err = cartoFacade.Stop(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)

		carto.StopFunc = func() error {
			return errors.New("test error 2")
		}
		cartoFacade.carto = &carto

		// returns error
		err = cartoFacade.Stop(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("test error 2"))

		carto.StopFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto

		// times out
		err = cartoFacade.Stop(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := errors.Join(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestTerminate(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor")
	algoCfg := GetTestAlgoConfig()
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	carto.TerminateFunc = func() error {
		return nil
	}
	cartoFacade.carto = &carto
	cartoFacade.Initialize(cancelCtx, 5*time.Second, &activeBackgroundWorkers)
	cartoFacade.Terminate(cancelCtx, 5*time.Second)

	t.Run("testing Terminate", func(t *testing.T) {
		// success case
		err = cartoFacade.Terminate(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)

		carto.TerminateFunc = func() error {
			return errors.New("test error 3")
		}
		cartoFacade.carto = &carto

		// returns error
		err = cartoFacade.Terminate(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("test error 3"))

		carto.TerminateFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto

		// times out
		err = cartoFacade.Terminate(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := errors.Join(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestAddSensorReading(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor")
	algoCfg := GetTestAlgoConfig()
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	carto.AddSensorReadingFunc = func(name string, reading []byte, time time.Time) error {
		return nil
	}
	cartoFacade.carto = &carto
	cartoFacade.Initialize(cancelCtx, 5*time.Second, &activeBackgroundWorkers)
	cartoFacade.Terminate(cancelCtx, 5*time.Second)

	t.Run("testing AddSensorReading", func(t *testing.T) {
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)

		// read PCD
		file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/0.pcd"))
		test.That(t, err, test.ShouldBeNil)
		buf := new(bytes.Buffer)
		pc, err := pointcloud.ReadPCD(file)
		test.That(t, err, test.ShouldBeNil)
		err = pointcloud.ToPCD(pc, buf, 0)
		test.That(t, err, test.ShouldBeNil)

		// success case
		err = cartoFacade.AddSensorReading(cancelCtx, 5*time.Second, "mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeNil)

		carto.AddSensorReadingFunc = func(name string, reading []byte, time time.Time) error {
			return errors.New("test error 4")
		}
		cartoFacade.carto = &carto

		// returns error
		err = cartoFacade.AddSensorReading(cancelCtx, 5*time.Second, "mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("test error 4"))

		carto.AddSensorReadingFunc = func(name string, reading []byte, timestamp time.Time) error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}
		cartoFacade.carto = &carto

		// times out
		err = cartoFacade.AddSensorReading(cancelCtx, 1*time.Millisecond, "mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeError)
		expectedErr := errors.Join(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestGetPosition(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor")
	algoCfg := GetTestAlgoConfig()
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	carto.GetPositionFunc = func() (GetPosition, error) {
		pos := GetPosition{X: 1, Y: 2, Z: 3}
		return pos, nil
	}
	cartoFacade.carto = &carto
	cartoFacade.Initialize(cancelCtx, 5*time.Second, &activeBackgroundWorkers)
	cartoFacade.Terminate(cancelCtx, 5*time.Second)

	t.Run("testing GetPosition", func(t *testing.T) {
		// success case
		pos, err := cartoFacade.GetPosition(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pos.X, test.ShouldEqual, 1)
		test.That(t, pos.Y, test.ShouldEqual, 2)
		test.That(t, pos.Z, test.ShouldEqual, 3)

		carto.GetPositionFunc = func() (GetPosition, error) {
			return GetPosition{}, errors.New("test error 5")
		}
		cartoFacade.carto = &carto

		// returns error
		_, err = cartoFacade.GetPosition(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("test error 5"))

		carto.GetPositionFunc = func() (GetPosition, error) {
			time.Sleep(50 * time.Millisecond)
			return GetPosition{}, nil
		}
		cartoFacade.carto = &carto

		// times out
		_, err = cartoFacade.GetPosition(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := errors.Join(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestGetInternalState(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor")
	algoCfg := GetTestAlgoConfig()
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	carto.GetInternalStateFunc = func() ([]byte, error) {
		internalState := []byte("hello!")
		return internalState, nil
	}
	cartoFacade.carto = &carto
	cartoFacade.Initialize(cancelCtx, 5*time.Second, &activeBackgroundWorkers)
	cartoFacade.Terminate(cancelCtx, 5*time.Second)

	t.Run("testing GetInternalState", func(t *testing.T) {
		// success case
		internalState, err := cartoFacade.GetInternalState(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldResemble, []byte("hello!"))

		carto.GetInternalStateFunc = func() ([]byte, error) {
			return []byte{}, errors.New("test error 6")
		}
		cartoFacade.carto = &carto

		// returns error
		_, err = cartoFacade.GetInternalState(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("test error 6"))

		carto.GetInternalStateFunc = func() ([]byte, error) {
			time.Sleep(50 * time.Millisecond)
			return []byte{}, nil
		}
		cartoFacade.carto = &carto

		// times out
		_, err = cartoFacade.GetInternalState(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := errors.Join(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}

func TestGetPointCloudMap(t *testing.T) {
	lib := CartoLibMock{}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	activeBackgroundWorkers := sync.WaitGroup{}

	cfg, dir, err := GetTestConfig("mysensor")
	algoCfg := GetTestAlgoConfig()
	test.That(t, err, test.ShouldBeNil)
	defer os.RemoveAll(dir)

	cartoFacade := New(&lib, cfg, algoCfg)
	carto := CartoMock{}
	carto.GetPointCloudMapFunc = func() ([]byte, error) {
		internalState := []byte("hello!")
		return internalState, nil
	}
	cartoFacade.carto = &carto
	cartoFacade.Initialize(cancelCtx, 5*time.Second, &activeBackgroundWorkers)
	cartoFacade.Terminate(cancelCtx, 5*time.Second)

	t.Run("testing GetPointCloudMap", func(t *testing.T) {
		// success case
		internalState, err := cartoFacade.GetPointCloudMap(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldResemble, []byte("hello!"))

		carto.GetPointCloudMapFunc = func() ([]byte, error) {
			return []byte{}, errors.New("test error 7")
		}
		cartoFacade.carto = &carto

		// returns error
		_, err = cartoFacade.GetPointCloudMap(cancelCtx, 5*time.Second)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("test error 7"))

		carto.GetPointCloudMapFunc = func() ([]byte, error) {
			time.Sleep(50 * time.Millisecond)
			return []byte{}, nil
		}
		cartoFacade.carto = &carto

		// times out
		_, err = cartoFacade.GetPointCloudMap(cancelCtx, 1*time.Millisecond)
		test.That(t, err, test.ShouldBeError)
		expectedErr := errors.Join(errors.New(timeoutErrMessage), context.DeadlineExceeded)
		test.That(t, err, test.ShouldResemble, expectedErr)
	})

	cancelFunc()
	activeBackgroundWorkers.Wait()
}
