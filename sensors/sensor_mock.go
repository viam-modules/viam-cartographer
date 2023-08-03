package sensors

import (
	"context"
)

// TimedLidarSensorMock represents a fake TimedLidarSensor.
type TimedLidarSensorMock struct {
	TimedLidarSensorReadingFunc func(ctx context.Context) (TimedLidarSensorReadingResponse, error)
}

// TimedLidarSensorMock represents a fake TimedLidarSensor.
type TimedIMUSensorMock struct {
	TimedIMUSensorReadingFunc func(ctx context.Context) (TimedIMUSensorReadingResponse, error)
}

// TimedLidarSensorReading returns a fake TimedLidarSensorReadingResponse or an error
// panics if TimedLidarSensorReadingFunc is nil.
func (tsm *TimedLidarSensorMock) TimedLidarSensorReading(ctx context.Context) (TimedLidarSensorReadingResponse, error) {
	return tsm.TimedLidarSensorReadingFunc(ctx)
}

// TimedLidarSensorReading returns a fake TimedLidarSensorReadingResponse or an error
// panics if TimedLidarSensorReadingFunc is nil.
func (tsm *TimedIMUSensorMock) TimedIMUSensorReading(ctx context.Context) (TimedIMUSensorReadingResponse, error) {
	return tsm.TimedIMUSensorReadingFunc(ctx)
}
