// Package inject is used to mock a cartofacade.
package inject

import (
	"context"
	"sync"
	"time"

	queue "github.com/viamrobotics/viam-cartographer/cartofacade"
)

// WorkItem represents a fake instance of cartofacade.
type WorkItem struct {
	queue.WorkItem
	DoWorkFunc func(q *queue.Queue) (interface{}, queue.ResultType, error)
}

// DoWork calls the injected DoWorkFunc or the real version.
func (w *WorkItem) DoWork(q *queue.Queue) (interface{}, queue.ResultType, error) {
	if w.DoWorkFunc == nil {
		return w.WorkItem.DoWork(q)
	}
	return w.DoWorkFunc(q)
}

type Queue struct {
	queue.Queue
	RequestFunc               func(ctxParent context.Context, workType queue.WorkType, inputs map[queue.InputType]interface{}, timeout time.Duration) queue.Response
	StartBackgroundWorkerFunc func(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup)
}

// Request calls the injected RequestFunc or the real version.
func (q *Queue) Request(ctxParent context.Context, workType queue.WorkType, inputs map[queue.InputType]interface{}, timeout time.Duration) queue.Response {
	if q.RequestFunc == nil {
		return q.Queue.Request(ctxParent, workType, inputs, timeout)
	}
	return q.RequestFunc(ctxParent, workType, inputs, timeout)
}

// StartBackgroundWorker calls the injected StartBackgroundWorkerFunc or the real version.
func (q *Queue) StartBackgroundWorker(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup) {
	if q.StartBackgroundWorkerFunc == nil {
		q.Queue.StartBackgroundWorker(ctx, activeBackgroundWorkers)
	}
	q.StartBackgroundWorkerFunc(ctx, activeBackgroundWorkers)
}
