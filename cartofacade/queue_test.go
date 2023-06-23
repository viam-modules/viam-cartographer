package cartofacade

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"github.com/viamrobotics/viam-cartographer/cartofacade/inject"
	cgoApi "github.com/viamrobotics/viam-cartographer/cartofacade/internal/capi"
	"go.viam.com/test"
)

func TestDoWork(t *testing.T) {

}

func TestRequest(t *testing.T) {
	cartoLib := inject.CartoLib{}
	cartoLib.TerminateFunc = func() error {
		return nil
	}
	t.Run("test successful request", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, _, err := cgoApi.GetTestConfig()
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cgoApi.GetTestAlgoConfig()
		carto := inject.Carto{}
		carto.StartFunc = func() error {
			return nil
		}

		queue := NewQueue(&cartoLib, config, algoConfig)
		queue.Carto = &carto
		queue.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)

		result, err := queue.Request(cancelCtx, Start, map[InputType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, result.result, test.ShouldBeNil)
		test.That(t, result.resultType, test.ShouldEqual, Nil)

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("test failed request", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, _, err := cgoApi.GetTestConfig()
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cgoApi.GetTestAlgoConfig()
		carto := inject.Carto{}

		testErr := errors.New("error")
		carto.StartFunc = func() error {
			return testErr
		}

		queue := NewQueue(&cartoLib, config, algoConfig)
		queue.Carto = &carto
		queue.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)

		result, err := queue.Request(cancelCtx, Start, map[InputType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldResemble, testErr)
		test.That(t, result, test.ShouldResemble, Result{})

		cancelFunc()
		activeBackgroundWorkers.Wait()
	})

	t.Run("test requesting with a canceled context", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		activeBackgroundWorkers := sync.WaitGroup{}

		config, _, err := cgoApi.GetTestConfig()
		test.That(t, err, test.ShouldBeNil)

		algoConfig := cgoApi.GetTestAlgoConfig()
		carto := inject.Carto{}

		testErr := errors.New("error")
		carto.StartFunc = func() error {
			return testErr
		}

		queue := NewQueue(&cartoLib, config, algoConfig)
		queue.Carto = &carto
		queue.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)
		cancelFunc()
		activeBackgroundWorkers.Wait()

		result, err := queue.Request(cancelCtx, Start, map[InputType]interface{}{}, 5*time.Second)
		test.That(t, err, test.ShouldResemble, errors.New("timeout has occurred while trying to write request to cartofacade"))
		test.That(t, result, test.ShouldResemble, Result{})

	})

	err := cartoLib.Terminate()
	test.That(t, err, test.ShouldBeNil)
}
