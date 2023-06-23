// Package inject is used to mock a cartofacade.
package inject

import (
	queue "github.com/viamrobotics/viam-cartographer/cartofacade"
)

// WorkItem represents a fake instance of cartofacade.
type WorkItem struct {
	queue.WorkItem
	DoWorkFunc func(q *queue.Queue) (interface{}, queue.ResultType, error)
}

// Terminate calls the injected TerminateFunc or the real version.
func (w *WorkItem) DoWork(q *queue.Queue) (interface{}, queue.ResultType, error) {
	if w.DoWorkFunc == nil {
		return w.WorkItem.DoWork(q)
	}
	return w.DoWorkFunc(q)
}
