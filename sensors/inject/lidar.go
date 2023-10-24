// Package inject provides dependency injected structures for mocking interfaces.
package inject

import (
	"context"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

// TimedLidarSensor is an injected TimedLidarSensor.
type TimedLidarSensor struct {
	s.Lidar
	NameFunc                    func() string
	DataFrequencyHzFunc         func() int
	TimedLidarSensorReadingFunc func(ctx context.Context) (s.TimedLidarSensorReadingResponse, error)
}

// Name calls the injected Name or the real version.
func (tls *TimedLidarSensor) Name() string {
	if tls.NameFunc == nil {
		return tls.Lidar.Name()
	}
	return tls.NameFunc()
}

// DataFrequencyHz calls the injected DataFrequencyHz or the real version.
func (tls *TimedLidarSensor) DataFrequencyHz() int {
	if tls.DataFrequencyHzFunc == nil {
		return tls.Lidar.DataFrequencyHz()
	}
	return tls.DataFrequencyHzFunc()
}

// TimedLidarSensorReading calls the injected TimedLidarSensorReading or the real version.
func (tls *TimedLidarSensor) TimedLidarSensorReading(ctx context.Context) (s.TimedLidarSensorReadingResponse, error) {
	if tls.TimedLidarSensorReadingFunc == nil {
		return tls.Lidar.TimedLidarSensorReading(ctx)
	}
	return tls.TimedLidarSensorReadingFunc(ctx)
}
