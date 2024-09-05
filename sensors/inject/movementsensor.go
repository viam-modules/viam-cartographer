package inject

import (
	"context"

	s "github.com/viam-modules/viam-cartographer/sensors"
)

// TimedMovementSensor is an injected TimedMovementSensor.
type TimedMovementSensor struct {
	s.MovementSensor
	NameFunc                       func() string
	DataFrequencyHzFunc            func() int
	TimedMovementSensorReadingFunc func(ctx context.Context) (s.TimedMovementSensorReadingResponse, error)
	PropertiesFunc                 func() s.MovementSensorProperties
}

// Name calls the injected Name or the real version.
func (tms *TimedMovementSensor) Name() string {
	if tms.NameFunc == nil {
		return tms.MovementSensor.Name()
	}
	return tms.NameFunc()
}

// DataFrequencyHz calls the injected DataFrequencyHz or the real version.
func (tms *TimedMovementSensor) DataFrequencyHz() int {
	if tms.DataFrequencyHzFunc == nil {
		return tms.MovementSensor.DataFrequencyHz()
	}
	return tms.DataFrequencyHzFunc()
}

// TimedMovementSensorReading calls the injected TimedMovementSensorReading or the real version.
func (tms *TimedMovementSensor) TimedMovementSensorReading(ctx context.Context) (s.TimedMovementSensorReadingResponse, error) {
	if tms.TimedMovementSensorReadingFunc == nil {
		return tms.MovementSensor.TimedMovementSensorReading(ctx)
	}
	return tms.TimedMovementSensorReadingFunc(ctx)
}

// Properties calls the injected Properties or the real version.
func (tms *TimedMovementSensor) Properties() s.MovementSensorProperties {
	if tms.PropertiesFunc == nil {
		return tms.MovementSensor.Properties()
	}
	return tms.PropertiesFunc()
}
