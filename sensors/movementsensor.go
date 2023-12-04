// Package sensors defines interfaces for sensors used by viam cartographer
package sensors

import (
	"context"
	"time"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/rdk/utils/contextutils"
)

const (
	movementSensorReadingTimeToleranceMsec = 50 // Milliseconds
	replayTimestampErrorMessage            = "replay sensor timestamp parse RFC3339Nano error"
	timedMovementSensorReadingTimeout      = 5 * time.Second
)

var (
	defaultTime = time.Time{}
	// ErrMovementSensorNeitherIMUNorOdometer denotes that the provided movement sensor does neither support
	// an IMU nor a movement sensor.
	ErrMovementSensorNeitherIMUNorOdometer = errors.New("'movement_sensor' must either support both LinearAcceleration and " +
		"AngularVelocity, or both Position and Orientation")
	// ErrNoValidReadingObtained denotes that the attempt to obtain a valid IMU or odometer reading failed.
	ErrNoValidReadingObtained = errors.New("could not obtain a reading that satisfies the time tolerance requirement")
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
	TestIsReplaySensor    bool
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
	name               string
	dataFrequencyHz    int
	imuSupported       bool
	odometerSupported  bool
	sensor             movementsensor.MovementSensor
	testIsReplaySensor bool
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
				return TimedMovementSensorReadingResponse{}, errors.Wrap(timeoutCtx.Err(), "timed out getting IMU data")
			default:
				if timedIMUReadingResponse, err = ms.timedIMUReading(timeoutCtx, &angVel, &linAcc,
					&readingTimeAngularVel, &readingTimeLinearAcc); err != nil && !errors.Is(err, ErrNoValidReadingObtained) {
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
				return TimedMovementSensorReadingResponse{}, errors.Wrap(timeoutCtx.Err(), "timed out getting odometer data")
			default:
				if timedOdometerReadingResponse, err = ms.timedOdometerReading(timeoutCtx, position, &orientation,
					&readingTimePosition, &readingTimeOrientation); err != nil && !errors.Is(err, ErrNoValidReadingObtained) {
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
		TestIsReplaySensor:    ms.testIsReplaySensor,
	}, nil
}

func (ms *MovementSensor) timedIMUReading(ctx context.Context, angVel *spatialmath.AngularVelocity, linAcc *r3.Vector,
	readingTimeAngularVel, readingTimeLinearAcc *time.Time,
) (*TimedIMUReadingResponse, error) {
	var err error

	returnReadingIfTimestampsWithinTolerance := func(readingTimeAngularVel, readingTimeLinearAcc time.Time,
		angVel *spatialmath.AngularVelocity, linAcc *r3.Vector,
	) (TimedIMUReadingResponse, bool) {
		if readingTimeAngularVel.Sub(readingTimeLinearAcc).Abs().Milliseconds() < movementSensorReadingTimeToleranceMsec {
			return TimedIMUReadingResponse{
				LinearAcceleration: *linAcc,
				AngularVelocity: spatialmath.AngularVelocity{
					X: rdkutils.DegToRad(angVel.X),
					Y: rdkutils.DegToRad(angVel.Y),
					Z: rdkutils.DegToRad(angVel.Z),
				},
				ReadingTime: averageReadingTimes(readingTimeLinearAcc, readingTimeAngularVel),
			}, true
		}
		return TimedIMUReadingResponse{}, false
	}

	if *readingTimeLinearAcc == defaultTime || readingTimeLinearAcc.Sub(*readingTimeAngularVel).Milliseconds() < 0 {
		ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
		if *linAcc, err = ms.sensor.LinearAcceleration(ctxWithMetadata, make(map[string]interface{})); err != nil {
			return &TimedIMUReadingResponse{}, errors.Wrap(err, "could not obtain LinearAcceleration")
		}

		if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
			ms.testIsReplaySensor = true
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
			return &TimedIMUReadingResponse{}, errors.Wrap(err, "could not obtain AngularVelocity")
		}

		if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
			ms.testIsReplaySensor = true
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

	return nil, ErrNoValidReadingObtained
}

func (ms *MovementSensor) timedOdometerReading(ctx context.Context, position *geo.Point, orientation *spatialmath.Orientation,
	readingTimePosition, readingTimeOrientation *time.Time,
) (*TimedOdometerReadingResponse, error) {
	var err error

	returnReadingIfTimestampsWithinTolerance := func(readingTimePosition, readingTimeOrientation time.Time,
		position *geo.Point, orientation *spatialmath.Orientation,
	) (TimedOdometerReadingResponse, bool) {
		if readingTimeOrientation.Sub(readingTimePosition).Abs().Milliseconds() < movementSensorReadingTimeToleranceMsec {
			return TimedOdometerReadingResponse{
				Position:    position,
				Orientation: *orientation,
				ReadingTime: averageReadingTimes(readingTimePosition, readingTimeOrientation),
			}, true
		}
		return TimedOdometerReadingResponse{}, false
	}

	if *readingTimePosition == defaultTime || readingTimePosition.Sub(*readingTimeOrientation).Milliseconds() < 0 {
		ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
		if position, _, err = ms.sensor.Position(ctxWithMetadata, make(map[string]interface{})); err != nil {
			return &TimedOdometerReadingResponse{}, errors.Wrap(err, "could not obtain Position")
		}

		if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
			ms.testIsReplaySensor = true
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
			return &TimedOdometerReadingResponse{}, errors.Wrap(err, "could not obtain Orientation")
		}

		if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
			ms.testIsReplaySensor = true
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

	return nil, ErrNoValidReadingObtained
}

// Properties returns MovementSensorProperties, which holds information about whether or not an IMU
// and/or odometer are supported.
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
	logger logging.Logger,
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
		return &MovementSensor{}, ErrMovementSensorNeitherIMUNorOdometer
	}

	return &MovementSensor{
		name:              movementSensorName,
		dataFrequencyHz:   dataFrequencyHz,
		imuSupported:      imuSupported,
		odometerSupported: odometerSupported,
		sensor:            movementSensor,
	}, nil
}

func averageReadingTimes(a, b time.Time) time.Time {
	if b.Equal(a) {
		return a
	} else if b.After(a) {
		return a.Add(b.Sub(a) / 2)
	} else {
		return b.Add(a.Sub(b) / 2)
	}
}
