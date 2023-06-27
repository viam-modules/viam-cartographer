// Package cartofacade is used to mock a cartofacade.
package cartofacade

// RequestMock represents a fake instance of cartofacade.
type RequestMock struct {
	Request
	doWorkFunc func(cf *CartoFacade) (interface{}, error)
}

// DoWork calls the injected DoWorkFunc or the real version.
func (r *RequestMock) doWork(cf *CartoFacade) (interface{}, error) {
	if r.doWorkFunc == nil {
		return r.Request.doWork(cf)
	}
	return r.doWorkFunc(cf)
}
