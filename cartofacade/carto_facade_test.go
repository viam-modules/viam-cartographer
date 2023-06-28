package cartofacade

import (
	"context"
	"errors"
	"os"
	"sync"
	"testing"
	"time"

	"go.viam.com/test"
)

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
		errMessage := "timeout has occurred while trying to write request to cartofacade. Did you forget to call Start()?"
		writeTimeoutErr := errors.New(errMessage)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, writeTimeoutErr)
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
		readTimeoutErr := errors.New("timeout has occurred while trying to read request from cartofacade")
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, readTimeoutErr)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	err := cartoLib.Terminate()
	test.That(t, err, test.ShouldBeNil)
}
