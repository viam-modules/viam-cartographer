package sensors

import (
	"context"
)

// TimedSensorMock represents a fake TimedSensor.
type TimedSensorMock struct {
	TimedLidarSensorReadingFunc func(ctx context.Context) (TimedSensorReadingResponse, error)
	TimedIMUSensorReadingFunc   func(ctx context.Context) (TimedSensorReadingResponse, error)
}

// TimedLidarSensorReading returns a fake TimedSensorReadingResponse or an error
// panics if TimedLidarSensorReadingFunc is nil.
func (tsm *TimedSensorMock) TimedLidarSensorReading(ctx context.Context) (TimedSensorReadingResponse, error) {
	return tsm.TimedLidarSensorReadingFunc(ctx)
}

// TimedLidarSensorReading returns a fake TimedSensorReadingResponse or an error
// panics if TimedLidarSensorReadingFunc is nil.
func (tsm *TimedSensorMock) TimedIMUSensorReading(ctx context.Context) (TimedSensorReadingResponse, error) {
	return tsm.TimedIMUSensorReadingFunc(ctx)
}
