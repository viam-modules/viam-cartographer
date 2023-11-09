// Package sensors defines interfaces for sensors used by viam cartographer
package sensors

import (
	"context"
	"fmt"
	"math"
	"strings"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/components/movementsensor/replay"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/rdk/utils/contextutils"
	goutils "go.viam.com/utils"
)

const (
	replayTimeToleranceMsec           = 10 // Milliseconds
	replayTimestampErrorMessage       = "replay sensor timestamp parse RFC3339Nano error"
	timedMovementSensorReadingTimeout = 5 * time.Second
)

var (
	defaultTime = time.Time{}
	// ErrMovementSensorNeitherIMUNorOdometer deontes that the provided movement sensor does neither support
	// an IMU nor a movement sensor
	ERRMovementSensorNeitherIMUNorOdometer = errors.New("'movement_sensor' must either support both LinearAcceleration and " +
		"AngularVelocity, or both Position and Orientation")
)

// TimedMovementSensor describes a sensor that reports the time the reading is from & whether or not it is
// from a replay sensor.
type TimedMovementSensor interface {
	Name() string
	DataFrequencyHz() int
	TimedMovementSensorReading(ctx context.Context) (TimedMovementSensorReadingResponse, error)
	Properties() MovementSensorProperties
}

// MovementSensorProperties contains information whether or not an IMU and/or odometer are supported.
type MovementSensorProperties struct {
	IMUSupported      bool
	OdometerSupported bool
}

// TimedMovementSensorReadingResponse contains IMU and odometer sensor reading responses
// as well as information about whether or not the readings are from a replay movement sensor.
type TimedMovementSensorReadingResponse struct {
	TimedIMUResponse      *TimedIMUReadingResponse
	TimedOdometerResponse *TimedOdometerReadingResponse
	IsReplaySensor        bool
}

// TimedIMUReadingResponse represents an IMU sensor reading with a time.
type TimedIMUReadingResponse struct {
	AngularVelocity    spatialmath.AngularVelocity
	LinearAcceleration r3.Vector
	ReadingTime        time.Time
}

// TimedOdometerReadingResponse represents an odometer sensor reading with a time.
type TimedOdometerReadingResponse struct {
	Position    *geo.Point
	Orientation spatialmath.Orientation
	ReadingTime time.Time
}

// MovementSensor represents a movement sensor.
type MovementSensor struct {
	name              string
	dataFrequencyHz   int
	imuSupported      bool
	odometerSupported bool
	replay            bool
	sensor            movementsensor.MovementSensor
}

// Name returns the name of the movement sensor.
func (ms *MovementSensor) Name() string {
	return ms.name
}

// DataFrequencyHz returns the data rate in ms of the movement sensor.
func (ms *MovementSensor) DataFrequencyHz() int {
	return ms.dataFrequencyHz
}

// TimedMovementSensorReading returns data from the movement sensor and the time the reading is from & whether
// it was a replay sensor or not.
func (ms *MovementSensor) TimedMovementSensorReading(ctx context.Context) (TimedMovementSensorReadingResponse, error) {
	var (
		readingTimeAngularVel, readingTimeLinearAcc time.Time
		readingTimePosition, readingTimeOrientation time.Time
		angVel                                      spatialmath.AngularVelocity
		linAcc                                      r3.Vector
		position                                    *geo.Point
		orientation                                 spatialmath.Orientation
		timedIMUReadingResponse                     *TimedIMUReadingResponse
		timedOdometerReadingResponse                *TimedOdometerReadingResponse
		err                                         error
	)

	timeoutCtx, cancel := context.WithTimeout(ctx, timedMovementSensorReadingTimeout)
	defer cancel()
	if ms.imuSupported {
	imuLoop:
		for {
			select {
			case <-timeoutCtx.Done():
				return TimedMovementSensorReadingResponse{}, timeoutCtx.Err()
			default:
				if timedIMUReadingResponse, err = ms.timedIMUReading(timeoutCtx, &angVel, &linAcc,
					&readingTimeAngularVel, &readingTimeLinearAcc); err != nil {
					return TimedMovementSensorReadingResponse{}, err
				}
				if timedIMUReadingResponse != nil {
					break imuLoop
				}
			}
		}
	}
	if ms.odometerSupported {
	odometerLoop:
		for {
			select {
			case <-timeoutCtx.Done():
				return TimedMovementSensorReadingResponse{}, timeoutCtx.Err()
			default:
				if timedOdometerReadingResponse, err = ms.timedOdometerReading(timeoutCtx, position, &orientation,
					&readingTimePosition, &readingTimeOrientation); err != nil {
					return TimedMovementSensorReadingResponse{}, err
				}
				if timedOdometerReadingResponse != nil {
					break odometerLoop
				}
			}
		}
	}
	return TimedMovementSensorReadingResponse{
		TimedIMUResponse:      timedIMUReadingResponse,
		TimedOdometerResponse: timedOdometerReadingResponse,
		IsReplaySensor:        ms.replay,
	}, nil
}

func (ms *MovementSensor) timedIMUReading(ctx context.Context, angVel *spatialmath.AngularVelocity, linAcc *r3.Vector,
	readingTimeAngularVel, readingTimeLinearAcc *time.Time) (*TimedIMUReadingResponse, error) {
	var err error

	returnReadingIfTimestampsWithinTolerance := func(timeAngVel, timeAcc time.Time,
		angVel *spatialmath.AngularVelocity, linAcc *r3.Vector) (TimedIMUReadingResponse, bool) {
		if math.Abs(float64(readingTimeAngularVel.Sub(*readingTimeLinearAcc).Milliseconds())) < replayTimeToleranceMsec {
			return TimedIMUReadingResponse{
				LinearAcceleration: *linAcc,
				AngularVelocity: spatialmath.AngularVelocity{
					X: rdkutils.DegToRad(angVel.X),
					Y: rdkutils.DegToRad(angVel.Y),
					Z: rdkutils.DegToRad(angVel.Z),
				},
				ReadingTime: readingTimeLinearAcc.Add((readingTimeLinearAcc.Sub(*readingTimeAngularVel)).Abs() / 2),
			}, true
		}
		return TimedIMUReadingResponse{}, false
	}

	if *readingTimeLinearAcc == defaultTime || readingTimeLinearAcc.Sub(*readingTimeAngularVel).Milliseconds() < 0 {
		ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
		if *linAcc, err = ms.sensor.LinearAcceleration(ctxWithMetadata, make(map[string]interface{})); err != nil {
			return &TimedIMUReadingResponse{}, errors.Wrap(err, "LinearAcceleration error")
		}

		if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
			ms.replay = true
			if *readingTimeLinearAcc, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0]); err != nil {
				return &TimedIMUReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
			}
		} else {
			*readingTimeLinearAcc = time.Now().UTC()
		}
	}

	if response, ok := returnReadingIfTimestampsWithinTolerance(*readingTimeAngularVel, *readingTimeLinearAcc, angVel, linAcc); ok {
		return &response, nil
	}

	if *readingTimeAngularVel == defaultTime || readingTimeAngularVel.Sub(*readingTimeLinearAcc).Milliseconds() < 0 {
		ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
		if *angVel, err = ms.sensor.AngularVelocity(ctxWithMetadata, make(map[string]interface{})); err != nil {
			return &TimedIMUReadingResponse{}, errors.Wrap(err, "AngularVelocity error")
		}

		if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
			ms.replay = true
			if *readingTimeAngularVel, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0]); err != nil {
				return &TimedIMUReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
			}
		} else {
			*readingTimeAngularVel = time.Now().UTC()
		}
	}

	if response, ok := returnReadingIfTimestampsWithinTolerance(*readingTimeAngularVel, *readingTimeLinearAcc, angVel, linAcc); ok {
		return &response, nil
	}

	return nil, nil
}

func (ms *MovementSensor) timedOdometerReading(ctx context.Context, position *geo.Point, orientation *spatialmath.Orientation,
	readingTimePosition, readingTimeOrientation *time.Time) (*TimedOdometerReadingResponse, error) {
	var err error

	returnReadingIfTimestampsWithinTolerance := func(timePos, timeOrientation time.Time,
		position *geo.Point, orientation *spatialmath.Orientation) (TimedOdometerReadingResponse, bool) {
		if math.Abs(float64(readingTimeOrientation.Sub(*readingTimePosition).Milliseconds())) < replayTimeToleranceMsec {
			return TimedOdometerReadingResponse{
				Position:    position,
				Orientation: *orientation,
				ReadingTime: readingTimePosition.Add((readingTimePosition.Sub(*readingTimeOrientation) / 2).Abs()),
			}, true
		}
		return TimedOdometerReadingResponse{}, false
	}

	if *readingTimePosition == defaultTime || readingTimePosition.Sub(*readingTimeOrientation).Milliseconds() < 0 {
		ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
		if position, _, err = ms.sensor.Position(ctxWithMetadata, make(map[string]interface{})); err != nil {
			return &TimedOdometerReadingResponse{}, errors.Wrap(err, "Position error")
		}

		if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
			ms.replay = true
			if *readingTimePosition, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0]); err != nil {
				return &TimedOdometerReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
			}
		} else {
			*readingTimePosition = time.Now().UTC()
		}
	}

	if response, ok := returnReadingIfTimestampsWithinTolerance(*readingTimePosition, *readingTimeOrientation, position, orientation); ok {
		return &response, nil
	}

	if *readingTimeOrientation == defaultTime || readingTimeOrientation.Sub(*readingTimePosition).Milliseconds() < 0 {
		ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
		if *orientation, err = ms.sensor.Orientation(ctxWithMetadata, make(map[string]interface{})); err != nil {
			return &TimedOdometerReadingResponse{}, errors.Wrap(err, "Orientation error")
		}

		if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
			ms.replay = true
			if *readingTimeOrientation, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0]); err != nil {
				return &TimedOdometerReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
			}
		} else {
			*readingTimeOrientation = time.Now().UTC()
		}
	}

	if response, ok := returnReadingIfTimestampsWithinTolerance(*readingTimePosition, *readingTimeOrientation, position, orientation); ok {
		return &response, nil
	}

	return nil, nil
}

func (ms *MovementSensor) Properties() MovementSensorProperties {
	return MovementSensorProperties{
		IMUSupported:      ms.imuSupported,
		OdometerSupported: ms.odometerSupported,
	}
}

// NewMovementSensor returns a new movement sensor.
func NewMovementSensor(
	ctx context.Context,
	deps resource.Dependencies,
	movementSensorName string,
	dataFrequencyHz int,
	logger golog.Logger,
) (TimedMovementSensor, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::sensors::NewMovementSensor")
	defer span.End()
	if movementSensorName == "" {
		return &MovementSensor{}, nil
	}
	movementSensor, err := movementsensor.FromDependencies(deps, movementSensorName)
	if err != nil {
		return &MovementSensor{}, errors.Wrapf(err, "error getting movement sensor \"%v\" for slam service", movementSensorName)
	}

	properties, err := movementSensor.Properties(ctx, make(map[string]interface{}))
	if err != nil {
		return &MovementSensor{}, errors.Wrapf(err, "error getting movement sensor properties from \"%v\" for slam service", movementSensorName)
	}

	imuSupported := properties.LinearAccelerationSupported && properties.AngularVelocitySupported
	odometerSupported := properties.PositionSupported && properties.OrientationSupported

	// A movement sensor must be support either an IMU, or an odometer, or both.
	if !imuSupported && !odometerSupported {
		return &MovementSensor{}, ERRMovementSensorNeitherIMUNorOdometer
	}

	fmt.Println("NewMovementSensor --> imuSupported, odometerSupported: ", imuSupported, odometerSupported)

	return &MovementSensor{
		name:              movementSensorName,
		dataFrequencyHz:   dataFrequencyHz,
		imuSupported:      imuSupported,
		odometerSupported: odometerSupported,
		sensor:            movementSensor,
	}, nil
}

// ValidateGetMovementSensorData checks every sensorValidationIntervalSec if the provided movement sensor
// returned valid timed readings every sensorValidationIntervalSec
// until either success or sensorValidationMaxTimeoutSec has elapsed.
// Returns an error if at least one invalid reading was returned.
func ValidateGetMovementSensorData(
	ctx context.Context,
	ms TimedMovementSensor,
	sensorValidationMaxTimeout time.Duration,
	sensorValidationInterval time.Duration,
	logger golog.Logger,
) error {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::sensor::ValidateGetMovementSensorData")
	defer span.End()

	startTime := time.Now().UTC()

	for {
		_, err := ms.TimedMovementSensorReading(ctx)
		if err == nil {
			break
		}

		logger.Debugw("ValidateGetMovementSensorData hit error: ", "error", err)
		// if the sensor is a replay movement sensor with no data ready, allow validation to pass
		// offline mode will stop the mapping session if the sensor still has no data,
		// while online mode will continue mapping once data is found by the replay sensor
		if strings.Contains(err.Error(), replay.ErrEndOfDataset.Error()) {
			break
		}
		if time.Since(startTime) >= sensorValidationMaxTimeout {
			return errors.Wrap(err, "ValidateGetMovementSensorData timeout")
		}
		if !goutils.SelectContextOrWait(ctx, sensorValidationInterval) {
			return ctx.Err()
		}
	}

	return nil
}
