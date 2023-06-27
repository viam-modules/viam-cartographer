// Package inject is used to mock a cartofacade.
package inject

import (
	cartofacade "github.com/viamrobotics/viam-cartographer/cartofacade"
)

// Request represents a fake instance of cartofacade.
type Request struct {
	cartofacade.Request
	DoWorkFunc func(cf *cartofacade.CartoFacade) (interface{}, error)
}

// DoWork calls the injected DoWorkFunc or the real version.
func (r *Request) DoWork(cf *cartofacade.CartoFacade) (interface{}, error) {
	if r.DoWorkFunc == nil {
		return r.Request.DoWork(cf)
	}
	return r.DoWorkFunc(cf)
}
