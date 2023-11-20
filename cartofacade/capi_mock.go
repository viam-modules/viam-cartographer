// Package cartofacade is used to mock the CGo API.
package cartofacade

import (
	s "github.com/viamrobotics/viam-cartographer/sensors"
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
	StartFunc                func() error
	StopFunc                 func() error
	TerminateFunc            func() error
	AddLidarReadingFunc      func(string, s.TimedLidarReadingResponse) error
	AddIMUReadingFunc        func(string, s.TimedIMUReadingResponse) error
	PositionFunc             func() (Position, error)
	PointCloudMapFunc        func() ([]byte, error)
	InternalStateFunc        func() ([]byte, error)
	RunFinalOptimizationFunc func() error
}

// start calls the injected StartFunc or the real version.
func (cf *CartoMock) start() error {
	if cf.StartFunc == nil {
		return cf.Carto.start()
	}
	return cf.StartFunc()
}

// stop calls the injected StopFunc or the real version.
func (cf *CartoMock) stop() error {
	if cf.StopFunc == nil {
		return cf.Carto.stop()
	}
	return cf.StopFunc()
}

// terminate calls the injected TerminateFunc or the real version.
func (cf *CartoMock) terminate() error {
	if cf.TerminateFunc == nil {
		return cf.Carto.terminate()
	}
	return cf.TerminateFunc()
}

// addLidarReading calls the injected AddLidarReadingFunc or the real version.
func (cf *CartoMock) addLidarReading(lidar string, reading s.TimedLidarReadingResponse) error {
	if cf.AddLidarReadingFunc == nil {
		return cf.Carto.addLidarReading(lidar, reading)
	}
	return cf.AddLidarReadingFunc(lidar, reading)
}

// addIMUReading calls the injected AddIMUReadingFunc or the real version.
func (cf *CartoMock) addIMUReading(imu string, reading s.TimedIMUReadingResponse) error {
	if cf.AddIMUReadingFunc == nil {
		return cf.Carto.addIMUReading(imu, reading)
	}
	return cf.AddIMUReadingFunc(imu, reading)
}

// position calls the injected PositionFunc or the real version.
func (cf *CartoMock) position() (Position, error) {
	if cf.PositionFunc == nil {
		return cf.Carto.position()
	}
	return cf.PositionFunc()
}

// pointCloudMap calls the injected PointCloudMap or the real version.
func (cf *CartoMock) pointCloudMap() ([]byte, error) {
	if cf.PointCloudMapFunc == nil {
		return cf.Carto.pointCloudMap()
	}
	return cf.PointCloudMapFunc()
}

// internalState calls the injected InternalState or the real version.
func (cf *CartoMock) internalState() ([]byte, error) {
	if cf.InternalStateFunc == nil {
		return cf.Carto.internalState()
	}
	return cf.InternalStateFunc()
}

// runFinalOptimization calls the injected RunFinalOptimization or the real version.
func (cf *CartoMock) runFinalOptimization() error {
	if cf.RunFinalOptimizationFunc == nil {
		return cf.Carto.runFinalOptimization()
	}
	return cf.RunFinalOptimizationFunc()
}
