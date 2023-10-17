package sensors

import (
	"context"
)

// TimedLidarSensorMock represents a fake TimedLidarSensor.
type TimedLidarSensorMock struct {
	Lidar
	NameFunc                    func() string
	DataFrequencyHzFunc         func() int
	TimedLidarSensorReadingFunc func(ctx context.Context) (TimedLidarSensorReadingResponse, error)
}

// TimedIMUSensorMock represents a fake TimedIMUSensor.
type TimedIMUSensorMock struct {
	IMU
	NameFunc                  func() string
	DataFrequencyHzFunc       func() int
	TimedIMUSensorReadingFunc func(ctx context.Context) (TimedIMUSensorReadingResponse, error)
}

// TimedLidarSensorReading returns a fake TimedLidarSensorReadingResponse or an error
// panics if TimedLidarSensorReadingFunc is nil.
func (tsm *TimedLidarSensorMock) TimedLidarSensorReading(ctx context.Context) (TimedLidarSensorReadingResponse, error) {
	return tsm.TimedLidarSensorReadingFunc(ctx)
}

// Name returns a fake name if NameFunc is defined or the actual name.
func (tsm *TimedLidarSensorMock) Name() string {
	if tsm.NameFunc == nil {
		return tsm.Lidar.Name()
	}
	return tsm.NameFunc()
}

// DataFrequencyHz returns a fake data frequency if DataFrequencyFunc is defined or the actual data frequency.
func (tsm *TimedLidarSensorMock) DataFrequencyHz() int {
	if tsm.DataFrequencyHzFunc == nil {
		return tsm.Lidar.DataFrequencyHz()
	}
	return tsm.DataFrequencyHzFunc()
}

// TimedIMUSensorReading returns a fake TimedIMUSensorReadingResponse or an error
// panics if TimedIMUSensorReadingFunc is nil.
func (tsm *TimedIMUSensorMock) TimedIMUSensorReading(ctx context.Context) (TimedIMUSensorReadingResponse, error) {
	return tsm.TimedIMUSensorReadingFunc(ctx)
}

// Name returns a fake name if NameFunc is defined or the actual name.
func (tsm *TimedIMUSensorMock) Name() string {
	if tsm.NameFunc == nil {
		return tsm.IMU.Name()
	}
	return tsm.NameFunc()
}

// DataFrequencyHz returns a fake data frequency if DataFrequencyFunc is defined or the actual data frequency.
func (tsm *TimedIMUSensorMock) DataFrequencyHz() int {
	if tsm.DataFrequencyHzFunc == nil {
		return tsm.IMU.DataFrequencyHz()
	}
	return tsm.DataFrequencyHzFunc()
}
