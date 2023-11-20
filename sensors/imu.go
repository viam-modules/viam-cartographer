// Package sensors defines interfaces for sensors used by viam cartographer
package sensors

import (
	"context"
	"math"
	"strings"
	"time"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/components/movementsensor/replay"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/rdk/utils/contextutils"
	goutils "go.viam.com/utils"
)

const (
	replayTimeToleranceMsec     = 10 // Milliseconds
	replayTimestampErrorMessage = "replay sensor timestamp parse RFC3339Nano error"
)

var defaultTime = time.Time{}

// TimedIMUSensor describes a sensor that reports the time the reading is from & whether or not it is from a replay sensor.
type TimedIMUSensor interface {
	Name() string
	DataFrequencyHz() int
	TimedIMUSensorReading(ctx context.Context) (TimedIMUSensorReadingResponse, error)
}

// TimedIMUSensorReadingResponse represents an IMU sensor reading with a time & allows the caller to know if the reading is
// from a replay movement sensor. Currently replay movement sensor are not yet supported.
type TimedIMUSensorReadingResponse struct {
	LinearAcceleration r3.Vector
	AngularVelocity    spatialmath.AngularVelocity
	ReadingTime        time.Time
	Replay             bool
}

// IMU represents an IMU movement sensor.
type IMU struct {
	name            string
	dataFrequencyHz int
	IMU             movementsensor.MovementSensor
}

// Name returns the name of the IMU.
func (imu IMU) Name() string {
	return imu.name
}

// DataFrequencyHz returns the data rate in ms of the IMU.
func (imu IMU) DataFrequencyHz() int {
	return imu.dataFrequencyHz
}

// TimedIMUSensorReading returns data from the IMU movement sensor and the time the reading is from.
// IMU Sensors currently do not support replay capabilities.
func (imu IMU) TimedIMUSensorReading(ctx context.Context) (TimedIMUSensorReadingResponse, error) {
	replay := false

	var timeLinearAcc, timeAngularVel time.Time
	var linAcc r3.Vector
	var angVel spatialmath.AngularVelocity
	var err error
	for {
		select {
		case <-ctx.Done():
			return TimedIMUSensorReadingResponse{}, nil
		default:
			if timeLinearAcc == defaultTime || timeLinearAcc.Sub(timeAngularVel).Milliseconds() < 0 {
				ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
				linAcc, err = imu.IMU.LinearAcceleration(ctxWithMetadata, make(map[string]interface{}))
				if err != nil {
					msg := "LinearAcceleration error"
					return TimedIMUSensorReadingResponse{}, errors.Wrap(err, msg)
				}
				timeLinearAcc = time.Now().UTC()

				timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
				if ok {
					replay = true
					timeLinearAcc, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
					if err != nil {
						return TimedIMUSensorReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
					}
				}
			}

			if timeAngularVel == defaultTime || timeAngularVel.Sub(timeLinearAcc).Milliseconds() < 0 {
				ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
				angVel, err = imu.IMU.AngularVelocity(ctxWithMetadata, make(map[string]interface{}))
				if err != nil {
					msg := "AngularVelocity error"
					return TimedIMUSensorReadingResponse{}, errors.Wrap(err, msg)
				}
				timeAngularVel = time.Now().UTC()

				timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
				if ok {
					replay = true
					timeAngularVel, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
					if err != nil {
						return TimedIMUSensorReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
					}
				}
			}
			if math.Abs(float64(timeAngularVel.Sub(timeLinearAcc).Milliseconds())) < replayTimeToleranceMsec {
				return TimedIMUSensorReadingResponse{
					LinearAcceleration: linAcc,
					AngularVelocity: spatialmath.AngularVelocity{
						X: rdkutils.DegToRad(angVel.X),
						Y: rdkutils.DegToRad(angVel.Y),
						Z: rdkutils.DegToRad(angVel.Z),
					},
					ReadingTime: timeLinearAcc.Add(timeLinearAcc.Sub(timeAngularVel) / 2),
					Replay:      replay,
				}, nil
			}
		}
	}
}

// NewIMU returns a new IMU.
func NewIMU(
	ctx context.Context,
	deps resource.Dependencies,
	movementSensorName string,
	dataFrequencyHz int,
	logger logging.Logger,
) (TimedIMUSensor, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::sensors::NewIMU")
	defer span.End()
	if movementSensorName == "" {
		return IMU{}, nil
	}
	movementSensor, err := movementsensor.FromDependencies(deps, movementSensorName)
	if err != nil {
		return IMU{}, errors.Wrapf(err, "error getting movement sensor \"%v\" for slam service", movementSensorName)
	}

	// A movement_sensor used as an IMU must support LinearAcceleration and AngularVelocity.
	properties, err := movementSensor.Properties(ctx, make(map[string]interface{}))
	if err != nil {
		return IMU{}, errors.Wrapf(err, "error getting movement sensor properties from \"%v\" for slam service", movementSensorName)
	}
	if !(properties.LinearAccelerationSupported && properties.AngularVelocitySupported) {
		return IMU{}, errors.New("configuring IMU movement sensor error: " +
			"'movement_sensor' must support both LinearAcceleration and AngularVelocity")
	}

	return IMU{
		name:            movementSensorName,
		dataFrequencyHz: dataFrequencyHz,
		IMU:             movementSensor,
	}, nil
}

// ValidateGetIMUData checks every sensorValidationIntervalSec if the provided IMU
// returned valid timed readings every sensorValidationIntervalSec
// until either success or sensorValidationMaxTimeoutSec has elapsed.
// returns an error if at least one invalid reading was returned.
func ValidateGetIMUData(
	ctx context.Context,
	imu TimedIMUSensor,
	sensorValidationMaxTimeout time.Duration,
	sensorValidationInterval time.Duration,
	logger logging.Logger,
) error {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::sensor::ValidateGetIMUData")
	defer span.End()

	startTime := time.Now().UTC()

	for {
		_, err := imu.TimedIMUSensorReading(ctx)
		if err == nil {
			break
		}

		logger.Debugw("ValidateGetIMUData hit error: ", "error", err)
		// if the sensor is a replay imu with no data ready, allow validation to pass
		// offline mode will stop the mapping session if the sensor still has no data,
		// while online mode will continue mapping once data is found by the replay sensor
		if strings.Contains(err.Error(), replay.ErrEndOfDataset.Error()) {
			break
		}
		if time.Since(startTime) >= sensorValidationMaxTimeout {
			return errors.Wrap(err, "ValidateGetIMUData timeout")
		}
		if !goutils.SelectContextOrWait(ctx, sensorValidationInterval) {
			return ctx.Err()
		}
	}

	return nil
}
