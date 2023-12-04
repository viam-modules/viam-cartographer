// Package cartofacade is used to mock a cartofacade.
package cartofacade

import (
	"context"
	"sync"
	"time"

	s "github.com/viamrobotics/viam-cartographer/sensors"
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

// Mock represents a fake instance of cartofacade.
type Mock struct {
	CartoFacade
	requestFunc func(
		ctxParent context.Context,
		requestType RequestType,
		inputs map[RequestParamType]interface{},
		timeout time.Duration,
	) (interface{}, error)
	startCGoRoutineFunc func(
		ctx context.Context,
		activeBackgroundWorkers *sync.WaitGroup,
	)

	InitializeFunc func(
		ctx context.Context,
		timeout time.Duration, activeBackgroundWorkers *sync.WaitGroup,
	) (SlamMode, error)
	StartFunc func(
		ctx context.Context,
		timeout time.Duration,
	) error
	StopFunc func(
		ctx context.Context,
		timeout time.Duration,
	) error
	TerminateFunc func(
		ctx context.Context,
		timeout time.Duration,
	) error
	AddLidarReadingFunc func(
		ctx context.Context,
		timeout time.Duration,
		lidarName string,
		currentReading s.TimedLidarReadingResponse,
	) error
	AddIMUReadingFunc func(
		ctx context.Context,
		timeout time.Duration,
		imuName string,
		currentReading s.TimedIMUReadingResponse,
	) error
	PositionFunc func(
		ctx context.Context,
		timeout time.Duration,
	) (Position, error)
	InternalStateFunc func(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error)
	PointCloudMapFunc func(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error)
}

// request calls the injected requestFunc or the real version.
func (cf *Mock) request(
	ctxParent context.Context,
	requestType RequestType,
	inputs map[RequestParamType]interface{},
	timeout time.Duration,
) (interface{}, error) {
	if cf.requestFunc == nil {
		return cf.CartoFacade.request(ctxParent, requestType, inputs, timeout)
	}
	return cf.requestFunc(ctxParent, requestType, inputs, timeout)
}

// start calls the injected startCGoRoutineFunc or the real version.
func (cf *Mock) startCGoroutine(
	ctx context.Context,
	activeBackgroundWorkers *sync.WaitGroup,
) {
	if cf.startCGoRoutineFunc == nil {
		cf.CartoFacade.startCGoroutine(ctx, activeBackgroundWorkers)
	}
	cf.startCGoRoutineFunc(ctx, activeBackgroundWorkers)
}

// Initialize calls the injected InitializeFunc or the real version.
func (cf *Mock) Initialize(
	ctx context.Context,
	timeout time.Duration,
	activeBackgroundWorkers *sync.WaitGroup,
) (SlamMode, error) {
	if cf.InitializeFunc == nil {
		return cf.CartoFacade.Initialize(ctx, timeout, activeBackgroundWorkers)
	}
	return cf.InitializeFunc(ctx, timeout, activeBackgroundWorkers)
}

// Start calls the injected StartFunc or the real version.
func (cf *Mock) Start(
	ctx context.Context,
	timeout time.Duration,
) error {
	if cf.StartFunc == nil {
		return cf.CartoFacade.Start(ctx, timeout)
	}
	return cf.StartFunc(ctx, timeout)
}

// Stop calls the Stop StopFunc or the real version.
func (cf *Mock) Stop(
	ctx context.Context,
	timeout time.Duration,
) error {
	if cf.StopFunc == nil {
		return cf.CartoFacade.Stop(ctx, timeout)
	}
	return cf.StopFunc(ctx, timeout)
}

// Terminate calls the injected TerminateFunc or the real version.
func (cf *Mock) Terminate(
	ctx context.Context,
	timeout time.Duration,
) error {
	if cf.TerminateFunc == nil {
		return cf.CartoFacade.Terminate(ctx, timeout)
	}
	return cf.TerminateFunc(ctx, timeout)
}

// AddLidarReading calls the injected AddLidarReadingFunc or the real version.
func (cf *Mock) AddLidarReading(
	ctx context.Context,
	timeout time.Duration,
	lidarName string,
	currentReading s.TimedLidarReadingResponse,
) error {
	if cf.AddLidarReadingFunc == nil {
		return cf.CartoFacade.AddLidarReading(ctx, timeout, lidarName, currentReading)
	}
	return cf.AddLidarReadingFunc(ctx, timeout, lidarName, currentReading)
}

// AddIMUReading calls the injected AddIMUReadingFunc or the real version.
func (cf *Mock) AddIMUReading(
	ctx context.Context,
	timeout time.Duration,
	imuName string,
	currentReading s.TimedIMUReadingResponse,
) error {
	if cf.AddIMUReadingFunc == nil {
		return cf.CartoFacade.AddIMUReading(ctx, timeout, imuName, currentReading)
	}
	return cf.AddIMUReadingFunc(ctx, timeout, imuName, currentReading)
}

// Position calls the injected PositionFunc or the real version.
func (cf *Mock) Position(
	ctx context.Context,
	timeout time.Duration,
) (Position, error) {
	if cf.PositionFunc == nil {
		return cf.CartoFacade.Position(ctx, timeout)
	}
	return cf.PositionFunc(ctx, timeout)
}

// InternalState calls the injected InternalStateFunc or the real version.
func (cf *Mock) InternalState(
	ctx context.Context,
	timeout time.Duration,
) ([]byte, error) {
	if cf.InternalStateFunc == nil {
		return cf.CartoFacade.InternalState(ctx, timeout)
	}
	return cf.InternalStateFunc(ctx, timeout)
}

// PointCloudMap calls the injected PointCloudMapFunc or the real version.
func (cf *Mock) PointCloudMap(
	ctx context.Context,
	timeout time.Duration,
) ([]byte, error) {
	if cf.PointCloudMapFunc == nil {
		return cf.CartoFacade.PointCloudMap(ctx, timeout)
	}
	return cf.PointCloudMapFunc(ctx, timeout)
}
