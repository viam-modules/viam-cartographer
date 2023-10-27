package inject

import (
	"context"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

// TimedOdometerSensor is an injected TimedOdometerSensor.
type TimedOdometerSensor struct {
	s.Odometer
	NameFunc                       func() string
	DataFrequencyHzFunc            func() int
	TimedOdometerSensorReadingFunc func(ctx context.Context) (s.TimedOdometerSensorReadingResponse, error)
}

// Name calls the injected Name or the real version.
func (tos *TimedOdometerSensor) Name() string {
	if tos.NameFunc == nil {
		return tos.Odometer.Name()
	}
	return tos.NameFunc()
}

// DataFrequencyHz calls the injected DataFrequencyHz or the real version.
func (tos *TimedOdometerSensor) DataFrequencyHz() int {
	if tos.DataFrequencyHzFunc == nil {
		return tos.Odometer.DataFrequencyHz()
	}
	return tos.DataFrequencyHzFunc()
}

// TimedOdometerSensorReading calls the injected TimedOdometerSensorReading or the real version.
func (tos *TimedOdometerSensor) TimedOdometerSensorReading(ctx context.Context) (s.TimedOdometerSensorReadingResponse, error) {
	if tos.TimedOdometerSensorReadingFunc == nil {
		return tos.Odometer.TimedOdometerSensorReading(ctx)
	}
	return tos.TimedOdometerSensorReadingFunc(ctx)
}
