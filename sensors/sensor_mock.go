package sensors

import (
	"context"
)

// TimedSensorMock represents a fake TimedSensor.
type TimedSensorMock struct {
	TimedSensorReadingFunc func(ctx context.Context) (TimedSensorReadingResponse, error)
}

// TimedSensorReading returns a fake TimedSensorReadingResponse or an error
// panics if TimedSensorReadingFunc is nil.
func (tsm *TimedSensorMock) TimedSensorReading(ctx context.Context) (TimedSensorReadingResponse, error) {
	return tsm.TimedSensorReadingFunc(ctx)
}
