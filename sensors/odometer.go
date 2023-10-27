// Package sensors defines interfaces for sensors used by viam cartographer
package sensors

import (
	"context"
	"math"
	"strings"
	"time"

	"github.com/edaniels/golog"
	geo "github.com/kellydunn/golang-geo"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/components/movementsensor/replay"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils/contextutils"
	goutils "go.viam.com/utils"
)

// TimedOdometerSensor describes a sensor that reports the time the reading is from & whether or not it is
// from a replay sensor.
type TimedOdometerSensor interface {
	Name() string
	DataFrequencyHz() int
	TimedOdometerSensorReading(ctx context.Context) (TimedOdometerSensorReadingResponse, error)
}

// TimedOdometerSensorReadingResponse represents an odometer sensor reading with a time & allows the caller
// to know if the reading is from a replay movement sensor.
type TimedOdometerSensorReadingResponse struct {
	Position    *geo.Point
	Orientation spatialmath.Orientation
	ReadingTime time.Time
	Replay      bool
}

// Odometer represents an Odometer movement sensor.
type Odometer struct {
	name            string
	dataFrequencyHz int
	Odometer        movementsensor.MovementSensor
}

// Name returns the name of the Odometer.
func (odom Odometer) Name() string {
	return odom.name
}

// DataFrequencyHz returns the data rate in ms of the Odometer.
func (odom Odometer) DataFrequencyHz() int {
	return odom.dataFrequencyHz
}

// TimedOdometerSensorReading returns data from the odometer movement sensor and the time the reading is from.
func (odom Odometer) TimedOdometerSensorReading(ctx context.Context) (TimedOdometerSensorReadingResponse, error) {
	replay := false

	var readingTimePosition, readingTimeOrientation time.Time
	var position *geo.Point
	var orientation spatialmath.Orientation
	var err error
	for {
		select {
		case <-ctx.Done():
			return TimedOdometerSensorReadingResponse{}, nil
		default:
			if readingTimePosition == defaultTime || readingTimePosition.Sub(readingTimeOrientation).Milliseconds() < 0 {
				ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
				if position, _, err = odom.Odometer.Position(ctxWithMetadata, make(map[string]interface{})); err != nil {
					msg := "Position error"
					return TimedOdometerSensorReadingResponse{}, errors.Wrap(err, msg)
				}

				readingTimePosition = time.Now().UTC()
				if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
					replay = true
					if readingTimePosition, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0]); err != nil {
						return TimedOdometerSensorReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
					}
				}
			}

			if readingTimeOrientation == defaultTime || readingTimeOrientation.Sub(readingTimePosition).Milliseconds() < 0 {
				ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
				if orientation, err = odom.Odometer.Orientation(ctxWithMetadata, make(map[string]interface{})); err != nil {
					msg := "Orientation error"
					return TimedOdometerSensorReadingResponse{}, errors.Wrap(err, msg)
				}

				readingTimeOrientation = time.Now().UTC()
				if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
					replay = true
					if readingTimeOrientation, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0]); err != nil {
						return TimedOdometerSensorReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
					}
				}
			}
			if math.Abs(float64(readingTimeOrientation.Sub(readingTimePosition).Milliseconds())) < replayTimeToleranceMsec {
				return TimedOdometerSensorReadingResponse{
					Position:    position,
					Orientation: orientation,
					ReadingTime: readingTimePosition.Add(readingTimePosition.Sub(readingTimeOrientation) / 2),
					Replay:      replay,
				}, nil
			}
		}
	}
}

// NewOdometer returns a new Odometer.
func NewOdometer(
	ctx context.Context,
	deps resource.Dependencies,
	movementSensorName string,
	dataFrequencyHz int,
	logger golog.Logger,
) (TimedOdometerSensor, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::sensors::NewOdometer")
	defer span.End()
	if movementSensorName == "" {
		return Odometer{}, nil
	}
	movementSensor, err := movementsensor.FromDependencies(deps, movementSensorName)
	if err != nil {
		return Odometer{}, errors.Wrapf(err, "error getting movement sensor \"%v\" for slam service", movementSensorName)
	}

	// A movement_sensor used as an Odometer must support Position and Orientation.
	properties, err := movementSensor.Properties(ctx, make(map[string]interface{}))
	if err != nil {
		return Odometer{}, errors.Wrapf(err, "error getting movement sensor properties from \"%v\" for slam service", movementSensorName)
	}
	if !(properties.PositionSupported && properties.OrientationSupported) {
		return Odometer{}, errors.New("configuring Odometer movement sensor error: " +
			"'movement_sensor' must support both Position and Orientation")
	}

	return Odometer{
		name:            movementSensorName,
		dataFrequencyHz: dataFrequencyHz,
		Odometer:        movementSensor,
	}, nil
}

// ValidateGetOdometerData checks every sensorValidationIntervalSec if the provided Odometer
// returned valid timed readings every sensorValidationIntervalSec
// until either success or sensorValidationMaxTimeoutSec has elapsed.
// returns an error if at least one invalid reading was returned.
func ValidateGetOdometerData(
	ctx context.Context,
	odom TimedOdometerSensor,
	sensorValidationMaxTimeout time.Duration,
	sensorValidationInterval time.Duration,
	logger golog.Logger,
) error {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::sensor::ValidateGetOdometerData")
	defer span.End()

	startTime := time.Now().UTC()

	for {
		_, err := odom.TimedOdometerSensorReading(ctx)
		if err == nil {
			break
		}

		logger.Debugw("ValidateGetOdometerData hit error: ", "error", err)
		// if the sensor is a replay odometer with no data ready, allow validation to pass
		// offline mode will stop the mapping session if the sensor still has no data,
		// while online mode will continue mapping once data is found by the replay sensor
		if strings.Contains(err.Error(), replay.ErrEndOfDataset.Error()) {
			break
		}
		if time.Since(startTime) >= sensorValidationMaxTimeout {
			return errors.Wrap(err, "ValidateGetOdometerData timeout")
		}
		if !goutils.SelectContextOrWait(ctx, sensorValidationInterval) {
			return ctx.Err()
		}
	}

	return nil
}
