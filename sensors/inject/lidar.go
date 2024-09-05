// Package inject provides dependency injected structures for mocking interfaces.
package inject

import (
	"context"

	s "github.com/viam-modules/viam-cartographer/sensors"
)

// TimedLidar is an injected TimedLidar.
type TimedLidar struct {
	s.Lidar
	NameFunc              func() string
	DataFrequencyHzFunc   func() int
	TimedLidarReadingFunc func(ctx context.Context) (s.TimedLidarReadingResponse, error)
}

// Name calls the injected Name or the real version.
func (tls *TimedLidar) Name() string {
	if tls.NameFunc == nil {
		return tls.Lidar.Name()
	}
	return tls.NameFunc()
}

// DataFrequencyHz calls the injected DataFrequencyHz or the real version.
func (tls *TimedLidar) DataFrequencyHz() int {
	if tls.DataFrequencyHzFunc == nil {
		return tls.Lidar.DataFrequencyHz()
	}
	return tls.DataFrequencyHzFunc()
}

// TimedLidarReading calls the injected TimedLidarReading or the real version.
func (tls *TimedLidar) TimedLidarReading(ctx context.Context) (s.TimedLidarReadingResponse, error) {
	if tls.TimedLidarReadingFunc == nil {
		return tls.Lidar.TimedLidarReading(ctx)
	}
	return tls.TimedLidarReadingFunc(ctx)
}
