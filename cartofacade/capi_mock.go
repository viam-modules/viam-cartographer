// Package cartofacade is used to mock the CGo API.
package cartofacade

import (
	"time"
)

// CartoLibMock represents a fake instance of cartofacade.
type CartoLibMock struct {
	CartoLib
	TerminateFunc func() error
}

// Terminate calls the injected TerminateFunc or the real version.
func (cf *CartoLibMock) Terminate() error {
	if cf.TerminateFunc == nil {
		return cf.CartoLib.Terminate()
	}
	return cf.TerminateFunc()
}

// CartoMock represents a fake instance of cartofacade.
type CartoMock struct {
	Carto
	StartFunc            func() error
	StopFunc             func() error
	TerminateFunc        func() error
	AddLidarReadingFunc  func(string, []byte, time.Time) error
	AddIMUReadingFunc    func(string, IMUReading, time.Time) error
	GetPositionFunc      func() (GetPosition, error)
	GetPointCloudMapFunc func() ([]byte, error)
	GetInternalStateFunc func() ([]byte, error)
}

// Start calls the injected StartFunc or the real version.
func (cf *CartoMock) start() error {
	if cf.StartFunc == nil {
		return cf.Carto.start()
	}
	return cf.StartFunc()
}

// Stop calls the injected StopFunc or the real version.
func (cf *CartoMock) stop() error {
	if cf.StopFunc == nil {
		return cf.Carto.stop()
	}
	return cf.StopFunc()
}

// Terminate calls the injected TerminateFunc or the real version.
func (cf *CartoMock) terminate() error {
	if cf.TerminateFunc == nil {
		return cf.Carto.terminate()
	}
	return cf.TerminateFunc()
}

// addLidarReading calls the injected AddLidarReadingFunc or the real version.
func (cf *CartoMock) addLidarReading(lidar string, readings []byte, time time.Time) error {
	if cf.AddLidarReadingFunc == nil {
		return cf.Carto.addLidarReading(lidar, readings, time)
	}
	return cf.AddLidarReadingFunc(lidar, readings, time)
}

// addIMUReading calls the injected AddIMUReadingFunc or the real version.
func (cf *CartoMock) addIMUReading(imu string, readings IMUReading, time time.Time) error {
	if cf.AddIMUReadingFunc == nil {
		return cf.Carto.addIMUReading(imu, readings, time)
	}
	return cf.AddIMUReadingFunc(imu, readings, time)
}

// GetPosition calls the injected GetPositionFunc or the real version.
func (cf *CartoMock) getPosition() (GetPosition, error) {
	if cf.GetPositionFunc == nil {
		return cf.Carto.getPosition()
	}
	return cf.GetPositionFunc()
}

// GetPointCloudMap calls the injected GetPointCloudMap or the real version.
func (cf *CartoMock) getPointCloudMap() ([]byte, error) {
	if cf.GetPointCloudMapFunc == nil {
		return cf.Carto.getPointCloudMap()
	}
	return cf.GetPointCloudMapFunc()
}

// GetInternalState calls the injected GetInternalState or the real version.
func (cf *CartoMock) getInternalState() ([]byte, error) {
	if cf.GetInternalStateFunc == nil {
		return cf.Carto.getInternalState()
	}
	return cf.GetInternalStateFunc()
}
