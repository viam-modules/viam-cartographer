package sensors

import (
	"context"
)

// TimedSensorMock represents a fake TimedSensor.
type TimedSensorMock struct {
	TimedLidarSensorReadingFunc func(ctx context.Context) (TimedLidarSensorReadingResponse, error)
	TimedIMUSensorReadingFunc   func(ctx context.Context) (TimedLidarSensorReadingResponse, error)
}

// TimedLidarSensorReading returns a fake TimedLidarSensorReadingResponse or an error
// panics if TimedLidarSensorReadingFunc is nil.
func (tsm *TimedSensorMock) TimedLidarSensorReading(ctx context.Context) (TimedLidarSensorReadingResponse, error) {
	return tsm.TimedLidarSensorReadingFunc(ctx)
}

// TimedLidarSensorReading returns a fake TimedLidarSensorReadingResponse or an error
// panics if TimedLidarSensorReadingFunc is nil.
func (tsm *TimedSensorMock) TimedIMUSensorReading(ctx context.Context) (TimedLidarSensorReadingResponse, error) {
	return tsm.TimedIMUSensorReadingFunc(ctx)
}
