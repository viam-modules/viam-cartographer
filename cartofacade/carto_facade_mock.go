// Package cartofacade is used to mock a cartofacade.
package cartofacade

import (
	"context"
	"sync"
	"time"
)

// RequestMock represents a fake instance of a request.
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

// CartoFacadeMock represents a fake instance of cartofacade.
type CartoFacadeMock struct {
	CartoFacade
	requestFunc func(ctxParent context.Context, requestType RequestType, inputs map[RequestParamType]interface{}, timeout time.Duration) (interface{}, error)
	startFunc   func(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup)

	InitializeFunc       func(ctx context.Context, timeout time.Duration, activeBackgroundWorkers *sync.WaitGroup) error
	StartFunc            func(ctx context.Context, timeout time.Duration) error
	StopFunc             func(ctx context.Context, timeout time.Duration) error
	TerminateFunc        func(ctx context.Context, timeout time.Duration) error
	AddSensorReadingFunc func(ctx context.Context, timeout time.Duration, sensorName string, currentReading []byte, readingTimestamp time.Time) error
	GetPositionFunc      func(ctx context.Context, timeout time.Duration) (GetPosition, error)
	GetInternalStateFunc func(ctx context.Context, timeout time.Duration) ([]byte, error)
	GetPointCloudMapFunc func(ctx context.Context, timeout time.Duration) ([]byte, error)
}

// requestFunc calls the injected requestFunc or the real version.
func (cf *CartoFacadeMock) request(ctxParent context.Context, requestType RequestType, inputs map[RequestParamType]interface{}, timeout time.Duration) (interface{}, error) {
	if cf.requestFunc == nil {
		return cf.CartoFacade.request(ctxParent, requestType, inputs, timeout)
	}
	return cf.requestFunc(ctxParent, requestType, inputs, timeout)
}

// startFunc calls the injected startFunc or the real version.
func (cf *CartoFacadeMock) start(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup) {
	if cf.startFunc == nil {
		cf.CartoFacade.start(ctx, activeBackgroundWorkers)
	}
	cf.startFunc(ctx, activeBackgroundWorkers)
}

// InitializeFunc calls the injected InitializeFunc or the real version.
func (cf *CartoFacadeMock) Initialize(ctx context.Context, timeout time.Duration, activeBackgroundWorkers *sync.WaitGroup) error {
	if cf.InitializeFunc == nil {
		return cf.CartoFacade.Initialize(ctx, timeout, activeBackgroundWorkers)
	}
	return cf.InitializeFunc(ctx, timeout, activeBackgroundWorkers)
}

// StartFunc calls the injected StartFunc or the real version.
func (cf *CartoFacadeMock) Start(ctx context.Context, timeout time.Duration) error {
	if cf.StartFunc == nil {
		return cf.CartoFacade.Start(ctx, timeout)
	}
	return cf.StartFunc(ctx, timeout)
}

// StopFunc calls the Stop StopFunc or the real version.
func (cf *CartoFacadeMock) Stop(ctx context.Context, timeout time.Duration) error {
	if cf.StopFunc == nil {
		return cf.CartoFacade.Stop(ctx, timeout)
	}
	return cf.StopFunc(ctx, timeout)
}

// TerminateFunc calls the injected TerminateFunc or the real version.
func (cf *CartoFacadeMock) Terminate(ctx context.Context, timeout time.Duration) error {
	if cf.TerminateFunc == nil {
		return cf.CartoFacade.Terminate(ctx, timeout)
	}
	return cf.TerminateFunc(ctx, timeout)
}

// AddSensorReadingFunc calls the injected AddSensorReadingFunc or the real version.
func (cf *CartoFacadeMock) AddSensorReading(ctx context.Context, timeout time.Duration, sensorName string, currentReading []byte, readingTimestamp time.Time) error {
	if cf.AddSensorReadingFunc == nil {
		return cf.CartoFacade.AddSensorReading(ctx, timeout, sensorName, currentReading, readingTimestamp)
	}
	return cf.AddSensorReadingFunc(ctx, timeout, sensorName, currentReading, readingTimestamp)
}

// GetPositionFunc calls the injected GetPositionFunc or the real version.
func (cf *CartoFacadeMock) GetPosition(ctx context.Context, timeout time.Duration) (GetPosition, error) {
	if cf.GetPositionFunc == nil {
		return cf.CartoFacade.GetPosition(ctx, timeout)
	}
	return cf.GetPositionFunc(ctx, timeout)
}

// GetInternalStateFunc calls the injected GetInternalStateFunc or the real version.
func (cf *CartoFacadeMock) GetInternalState(ctx context.Context, timeout time.Duration) ([]byte, error) {
	if cf.GetInternalStateFunc == nil {
		return cf.CartoFacade.GetInternalState(ctx, timeout)
	}
	return cf.GetInternalStateFunc(ctx, timeout)
}

// GetPointCloudMapFunc calls the injected GetPointCloudMapFunc or the real version.
func (cf *CartoFacadeMock) GetPointCloudMap(ctx context.Context, timeout time.Duration) ([]byte, error) {
	if cf.GetPointCloudMapFunc == nil {
		return cf.CartoFacade.GetPointCloudMap(ctx, timeout)
	}
	return cf.GetPointCloudMapFunc(ctx, timeout)
}
