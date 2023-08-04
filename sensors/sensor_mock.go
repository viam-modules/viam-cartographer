package sensors

import (
	"context"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// TimedLidarSensorMock represents a fake TimedLidarSensor.
type TimedLidarSensorMock struct {
	TimedLidarSensorReadingFunc func(ctx context.Context) (TimedLidarSensorReadingResponse, error)
}

// TimedIMUSensorMock represents a fake TimedIMUSensor.
type TimedIMUSensorMock struct {
	TimedIMUSensorReadingFunc  func(ctx context.Context) (TimedIMUSensorReadingResponse, error)
	MockLinearAccelerationData []r3.Vector
	MockAngularVelocityData    []spatialmath.AngularVelocity
}

// TimedLidarSensorReading returns a fake TimedLidarSensorReadingResponse or an error
// panics if TimedLidarSensorReadingFunc is nil.
func (tsm *TimedLidarSensorMock) TimedLidarSensorReading(ctx context.Context) (TimedLidarSensorReadingResponse, error) {
	return tsm.TimedLidarSensorReadingFunc(ctx)
}

// TimedIMUSensorReading returns a fake TimedIMUSensorReadingResponse or an error
// panics if TimedIMUSensorReadingFunc is nil.
func (tsm *TimedIMUSensorMock) TimedIMUSensorReading(ctx context.Context) (TimedIMUSensorReadingResponse, error) {
	return tsm.TimedIMUSensorReadingFunc(ctx)
}
