// Package inject is used to mock the CGo API.
package inject

import (
	"time"

	cgoApi "github.com/viamrobotics/viam-cartographer/cartofacade"
)

// CartoLib represents a fake instance of cartofacade.
type CartoLib struct {
	cgoApi.CartoLib
	TerminateFunc func() error
}

// Terminate calls the injected TerminateFunc or the real version.
func (cf *CartoLib) Terminate() error {
	if cf.TerminateFunc == nil {
		return cf.CartoLib.Terminate()
	}
	return cf.TerminateFunc()
}

// Carto represents a fake instance of cartofacade.
type Carto struct {
	cgoApi.Carto
	StartFunc            func() error
	StopFunc             func() error
	TerminateFunc        func() error
	AddSensorReadingFunc func(string, []byte, time.Time) error
	GetPositionFunc      func() (cgoApi.GetPosition, error)
	GetPointCloudMapFunc func() ([]byte, error)
	GetInternalStateFunc func() ([]byte, error)
}

// Start calls the injected StartFunc or the real version.
func (cf *Carto) Start() error {
	if cf.StartFunc == nil {
		return cf.Carto.Start()
	}
	return cf.StartFunc()
}

// Stop calls the injected StopFunc or the real version.
func (cf *Carto) Stop() error {
	if cf.StopFunc == nil {
		return cf.Carto.Stop()
	}
	return cf.StopFunc()
}

// Terminate calls the injected TerminateFunc or the real version.
func (cf *Carto) Terminate() error {
	if cf.TerminateFunc == nil {
		return cf.Carto.Terminate()
	}
	return cf.TerminateFunc()
}

// AddSensorReading calls the injected AddSensorReadingFunc or the real version.
func (cf *Carto) AddSensorReading(sensor string, readings []byte, time time.Time) error {
	if cf.AddSensorReadingFunc == nil {
		return cf.Carto.AddSensorReading(sensor, readings, time)
	}
	return cf.AddSensorReadingFunc(sensor, readings, time)
}

// GetPosition calls the injected GetPositionFunc or the real version.
func (cf *Carto) GetPosition() (cgoApi.GetPosition, error) {
	if cf.GetPositionFunc == nil {
		return cf.Carto.GetPosition()
	}
	return cf.GetPositionFunc()
}

// GetPointCloudMap calls the injected GetPointCloudMap or the real version.
func (cf *Carto) GetPointCloudMap() ([]byte, error) {
	if cf.GetPointCloudMapFunc == nil {
		return cf.Carto.GetPointCloudMap()
	}
	return cf.GetPointCloudMapFunc()
}

// GetInternalState calls the injected GetInternalState or the real version.
func (cf *Carto) GetInternalState() ([]byte, error) {
	if cf.GetInternalStateFunc == nil {
		return cf.Carto.GetInternalState()
	}
	return cf.GetInternalStateFunc()
}
