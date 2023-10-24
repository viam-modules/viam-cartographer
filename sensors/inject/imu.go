package inject

import (
	"context"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

// TimedIMUSensor is an injected TimedIMUSensor.
type TimedIMUSensor struct {
	s.IMU
	NameFunc                  func() string
	DataFrequencyHzFunc       func() int
	TimedIMUSensorReadingFunc func(ctx context.Context) (s.TimedIMUSensorReadingResponse, error)
}

// Name calls the injected Name or the real version.
func (tis *TimedIMUSensor) Name() string {
	if tis.NameFunc == nil {
		return tis.IMU.Name()
	}
	return tis.NameFunc()
}

// DataFrequencyHz calls the injected DataFrequencyHz or the real version.
func (tis *TimedIMUSensor) DataFrequencyHz() int {
	if tis.DataFrequencyHzFunc == nil {
		return tis.IMU.DataFrequencyHz()
	}
	return tis.DataFrequencyHzFunc()
}

// TimedIMUSensorReading calls the injected TimedIMUSensorReading or the real version.
func (tis *TimedIMUSensor) TimedIMUSensorReading(ctx context.Context) (s.TimedIMUSensorReadingResponse, error) {
	if tis.TimedIMUSensorReadingFunc == nil {
		return tis.IMU.TimedIMUSensorReading(ctx)
	}
	return tis.TimedIMUSensorReadingFunc(ctx)
}
