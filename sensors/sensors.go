// Package sensors defines interfaces for sensors used by viam cartographer
package sensors

import (
	"context"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	goutils "go.viam.com/utils"
)

// TimedSensorReadingResponse represents a sensor reading with a time & allows the caller to know if the reading is from a replay sensor.
type TimedSensorReadingResponse struct {
	Reading     []byte
	ReadingTime time.Time
	Replay      bool
}

// TimedSensor desribes a sensor that reports the time the reading is from & whether or not it is from a replay sensor.
type TimedSensor interface {
	TimedSensorReading(ctx context.Context) (TimedSensorReadingResponse, error)
}

// ValidateGetData checks every sensorValidationIntervalSec if the provided lidar
// returned a valid timed lidar readings every sensorValidationIntervalSec
// until either success or sensorValidationMaxTimeoutSec has elapsed.
// returns an error if no valid lidar readings were returned.
func ValidateGetData(
	ctx context.Context,
	sensor TimedSensor,
	sensorValidationMaxTimeout time.Duration,
	sensorValidationInterval time.Duration,
	logger golog.Logger,
) error {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::sensor::ValidateGetData")
	defer span.End()

	startTime := time.Now().UTC()

	for {
		_, err := sensor.TimedSensorReading(ctx)
		if err == nil {
			break
		}

		logger.Debugw("ValidateGetData hit error: ", "error", err)
		if time.Since(startTime) >= sensorValidationMaxTimeout {
			return errors.Wrap(err, "ValidateGetData timeout")
		}
		if !goutils.SelectContextOrWait(ctx, sensorValidationInterval) {
			return ctx.Err()
		}
	}

	return nil
}
