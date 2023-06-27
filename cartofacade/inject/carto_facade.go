// Package inject is used to mock a cartofacade.
package inject

import (
	cartofacade "github.com/viamrobotics/viam-cartographer/cartofacade"
)

// WorkItem represents a fake instance of cartofacade.
type WorkItem struct {
	cartofacade.WorkItem
	DoWorkFunc func(cf *cartofacade.CartoFacade) (interface{}, error)
}

// DoWork calls the injected DoWorkFunc or the real version.
func (w *WorkItem) DoWork(cf *cartofacade.CartoFacade) (interface{}, error) {
	if w.DoWorkFunc == nil {
		return w.WorkItem.DoWork(cf)
	}
	return w.DoWorkFunc(cf)
}
