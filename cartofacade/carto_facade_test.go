package cartofacade_test

import (
	"context"
	"errors"
	"os"
	"sync"
	"testing"
	"time"

	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/cartofacade/inject"
)

func TestRequest(t *testing.T) {
	cartoLib := inject.CartoLib{}
	cartoLib.TerminateFunc = func() error {
		return nil
	}
	testErr := errors.New("error")
	t.Run("test successful request", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := cartofacade.GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cartofacade.GetTestAlgoConfig()
		carto := inject.Carto{}
		carto.StartFunc = func() error {
			return nil
		}

		cf := cartofacade.New(&cartoLib, config, algoConfig)
		cf.Carto = &carto
		cf.Start(cancelCtx, &activeBackgroundWorkers)

		res, err := cf.Request(cancelCtx, cartofacade.Start, map[cartofacade.RequestParamType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, res, test.ShouldBeNil)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("test failed request", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := cartofacade.GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cartofacade.GetTestAlgoConfig()
		carto := inject.Carto{}

		carto.StartFunc = func() error {
			return testErr
		}

		cf := cartofacade.New(&cartoLib, config, algoConfig)
		cf.Carto = &carto
		cf.Start(cancelCtx, &activeBackgroundWorkers)

		_, err = cf.Request(cancelCtx, cartofacade.Start, map[cartofacade.RequestParamType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeError)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("test requesting with a canceled context", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := cartofacade.GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cartofacade.GetTestAlgoConfig()
		carto := inject.Carto{}

		carto.StartFunc = func() error {
			return testErr
		}

		cf := cartofacade.New(&cartoLib, config, algoConfig)
		cf.Carto = &carto
		cf.Start(cancelCtx, &activeBackgroundWorkers)
		cancelFunc()
		activeBackgroundWorkers.Wait()

		_, err = cf.Request(cancelCtx, cartofacade.Start, map[cartofacade.RequestParamType]interface{}{}, 5*time.Second)
		errMessage := "timeout has occurred while trying to write request to cartofacade. Did you forget to call cartoFacade.Start()?"
		writeTimeoutErr := errors.New(errMessage)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, writeTimeoutErr)
	})

	t.Run("test requesting with when the work function takes longer than the timeout", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, dir, err := cartofacade.GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cartofacade.GetTestAlgoConfig()
		carto := inject.Carto{}

		carto.StartFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}

		cf := cartofacade.New(&cartoLib, config, algoConfig)
		cf.Carto = &carto
		cf.Start(cancelCtx, &activeBackgroundWorkers)

		_, err = cf.Request(cancelCtx, cartofacade.Start, map[cartofacade.RequestParamType]interface{}{}, 10*time.Millisecond)
		readTimeoutErr := errors.New("timeout has occurred while trying to read request from cartofacade")
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, readTimeoutErr)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	err := cartoLib.Terminate()
	test.That(t, err, test.ShouldBeNil)
}
