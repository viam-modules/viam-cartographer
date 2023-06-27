package cartofacade_test

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"go.viam.com/test"

	cartofacade "github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/cartofacade/inject"
	cgoApi "github.com/viamrobotics/viam-cartographer/cartofacade/internal/capi"
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

		config, _, err := cgoApi.GetTestConfig("mysensor")
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cgoApi.GetTestAlgoConfig()
		carto := inject.Carto{}
		carto.StartFunc = func() error {
			return nil
		}

		cf := cartofacade.New(&cartoLib, config, algoConfig)
		cf.Carto = &carto
		cf.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)

		res, err := cf.Request(cancelCtx, cartofacade.Start, map[cartofacade.InputType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, res, test.ShouldBeNil)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("test failed request", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, _, err := cgoApi.GetTestConfig("mysensor")
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cgoApi.GetTestAlgoConfig()
		carto := inject.Carto{}

		carto.StartFunc = func() error {
			return testErr
		}

		cf := cartofacade.New(&cartoLib, config, algoConfig)
		cf.Carto = &carto
		cf.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)

		_, err = cf.Request(cancelCtx, cartofacade.Start, map[cartofacade.InputType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeError)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("test requesting with a canceled context", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, _, err := cgoApi.GetTestConfig("mysensor")
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cgoApi.GetTestAlgoConfig()
		carto := inject.Carto{}

		carto.StartFunc = func() error {
			return testErr
		}

		cf := cartofacade.New(&cartoLib, config, algoConfig)
		cf.Carto = &carto
		cf.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)
		cancelFunc()
		activeBackgroundWorkers.Wait()

		_, err = cf.Request(cancelCtx, cartofacade.Start, map[cartofacade.InputType]interface{}{}, 5*time.Second)
		testErr = errors.New("timeout has occurred while trying to write request to cartofacade. Did you start the background worker?")
		test.That(t, err, test.ShouldResemble, testErr)
	})

	t.Run("test requesting with when the work function takes longer than the timeout", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, _, err := cgoApi.GetTestConfig("mysensor")
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cgoApi.GetTestAlgoConfig()
		carto := inject.Carto{}

		carto.StartFunc = func() error {
			time.Sleep(50 * time.Millisecond)
			return nil
		}

		cf := cartofacade.New(&cartoLib, config, algoConfig)
		cf.Carto = &carto
		cf.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)

		_, err = cf.Request(cancelCtx, cartofacade.Start, map[cartofacade.InputType]interface{}{}, 10*time.Millisecond)
		testErr = errors.New("timeout has occurred while trying to read request from cartofacade")
		test.That(t, err, test.ShouldResemble, testErr)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	err := cartoLib.Terminate()
	test.That(t, err, test.ShouldBeNil)
}
