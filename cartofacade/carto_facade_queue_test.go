package cartofacade

import (
	"context"
	"runtime"
	"sync"
	"testing"
	"time"

	"go.viam.com/test"
)

var numCgoCalls = 0

func TestCartoFacadeQueue(t *testing.T) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	go countCgoCalls(cancelCtx)

	activeBackgroundWorkers := sync.WaitGroup{}

	cartoLib, err := NewLib(1, 1)
	defer cartoLib.Terminate()
	test.That(t, err, test.ShouldBeNil)

	cartoConfig, _, err := getTestConfig()
	test.That(t, err, test.ShouldBeNil)

	cartoAlgoConfig := getTestAlgoConfig()
	test.That(t, err, test.ShouldBeNil)

	t.Run("Spawning off goroutines should still call into C in serial", func(t *testing.T) {
		q := NewQueue(&cartoLib, cartoConfig, cartoAlgoConfig)
		q.StartBackgroundWorker(cancelCtx, &activeBackgroundWorkers)

		carto := q.HandleIncomingRequest(cancelCtx, Initialize, make(map[InputType]interface{}))
		q.Carto = carto.(Carto)
		test.That(t, numCgoCalls, test.ShouldEqual, 17)

		kickoffCgoCalls(cancelCtx, q)

		time.Sleep(2 * time.Second)
		cancelFunc()

		test.That(t, err, test.ShouldBeNil)
		test.That(t, numCgoCalls, test.ShouldEqual, 18)

		activeBackgroundWorkers.Wait()
	})
}

func kickoffCgoCalls(cancelCtx context.Context, q Queue) {
	for i := 0; i < 5; i++ {
		go q.HandleIncomingRequest(cancelCtx, testType, make(map[InputType]interface{}))
	}
}

func countCgoCalls(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
			currentNumCgoCalls := runtime.NumCgoCall()
			if currentNumCgoCalls != int64(numCgoCalls) {
				numCgoCalls = int(currentNumCgoCalls)
			}
		}
	}
}
