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

func TestDoWork(t *testing.T) {
}

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

		queue := cartofacade.NewQueue(&cartoLib, config, algoConfig)
		queue.Carto = &carto
		queue.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)

		resp := queue.Request(cancelCtx, cartofacade.Start, map[cartofacade.InputType]interface{}{}, 5*time.Second)
		test.That(t, resp.ResultType, test.ShouldResemble, cartofacade.Nil)
		test.That(t, resp.Result, test.ShouldBeNil)

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

		queue := cartofacade.NewQueue(&cartoLib, config, algoConfig)
		queue.Carto = &carto
		queue.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)

		resp := queue.Request(cancelCtx, cartofacade.Start, map[cartofacade.InputType]interface{}{}, 5*time.Second)
		test.That(t, resp.ResultType, test.ShouldEqual, cartofacade.Error)
		test.That(t, resp.Result.(error), test.ShouldResemble, testErr)

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

		queue := cartofacade.NewQueue(&cartoLib, config, algoConfig)
		queue.Carto = &carto
		queue.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)
		cancelFunc()
		activeBackgroundWorkers.Wait()

		resp := queue.Request(cancelCtx, cartofacade.Start, map[cartofacade.InputType]interface{}{}, 5*time.Second)
		testErr = errors.New("timeout has occurred while trying to write request to cartofacade. Did you start the background worker?")
		test.That(t, resp.ResultType, test.ShouldEqual, cartofacade.Error)
		test.That(t, resp.Result.(error), test.ShouldResemble, testErr)
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

		queue := cartofacade.NewQueue(&cartoLib, config, algoConfig)
		queue.Carto = &carto
		queue.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)

		resp := queue.Request(cancelCtx, cartofacade.Start, map[cartofacade.InputType]interface{}{}, 10*time.Millisecond)
		testErr = errors.New("timeout has occurred while trying to read request from cartofacade")
		test.That(t, resp.ResultType, test.ShouldEqual, cartofacade.Error)
		test.That(t, resp.Result.(error), test.ShouldResemble, testErr)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	err := cartoLib.Terminate()
	test.That(t, err, test.ShouldBeNil)
}
