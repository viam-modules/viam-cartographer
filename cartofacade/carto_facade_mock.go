// Package cartofacade is used to mock a cartofacade.
package cartofacade

// RequestMock represents a fake instance of cartofacade.
type RequestMock struct {
	Request
	DoWorkFunc func(cf *CartoFacade) (interface{}, error)
}

// DoWork calls the injected DoWorkFunc or the real version.
func (r *RequestMock) DoWork(cf *CartoFacade) (interface{}, error) {
	if r.DoWorkFunc == nil {
		return r.Request.DoWork(cf)
	}
	return r.DoWorkFunc(cf)
}
