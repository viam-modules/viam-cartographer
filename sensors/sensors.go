// Package sensors defines interfaces for sensors used by viam cartographer
package sensors

import (
	"bytes"
	"context"
	"math"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/rdk/utils/contextutils"
	goutils "go.viam.com/utils"
)

const (
	replayTimeTolerance         = 20 // Milliseconds
	replayTimestampErrorMessage = "replay sensor timestamp parse RFC3339Nano error"
)

var defaultTime = time.Time{}

// Lidar represents a LIDAR sensor.
type Lidar struct {
	Name  string
	lidar camera.Camera
}

// IMU represents an IMU movement sensor.
type IMU struct {
	Name string
	imu  movementsensor.MovementSensor
}

// TimedLidarSensorReadingResponse represents a lidar sensor reading with a time &
// allows the caller to know if the reading is from a replay camera sensor.
type TimedLidarSensorReadingResponse struct {
	Reading     []byte
	ReadingTime time.Time
	Replay      bool
}

// TimedLidarSensor describes a sensor that reports the time the reading is from & whether or not it is from a replay sensor.
type TimedLidarSensor interface {
	TimedLidarSensorReading(ctx context.Context) (TimedLidarSensorReadingResponse, error)
}

// TimedIMUSensorReadingResponse represents an IMU sensor reading with a time & allows the caller to know if the reading is
// from a replay movement sensor. Currently replay movement sensor are not yet supported.
type TimedIMUSensorReadingResponse struct {
	LinearAcceleration r3.Vector
	AngularVelocity    spatialmath.AngularVelocity
	ReadingTime        time.Time
	Replay             bool
}

// TimedIMUSensor describes a sensor that reports the time the reading is from & whether or not it is from a replay sensor.
type TimedIMUSensor interface {
	TimedIMUSensorReading(ctx context.Context) (TimedIMUSensorReadingResponse, error)
}

// NewLidar returns a new Lidar.
func NewLidar(
	ctx context.Context,
	deps resource.Dependencies,
	cameraName string,
	logger golog.Logger,
) (Lidar, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::sensors::NewLidar")
	defer span.End()
	newLidar, err := camera.FromDependencies(deps, cameraName)
	if err != nil {
		return Lidar{}, errors.Wrapf(err, "error getting lidar camera %v for slam service", cameraName)
	}

	// If there is a camera provided in the 'camera' field, we enforce that it supports PCD.
	properties, err := newLidar.Properties(ctx)
	if err != nil {
		return Lidar{}, errors.Wrapf(err, "error getting lidar camera properties %v for slam service", cameraName)
	}

	if !properties.SupportsPCD {
		return Lidar{}, errors.New("configuring lidar camera error: " +
			"'camera' must support PCD")
	}

	return Lidar{
		Name:  cameraName,
		lidar: newLidar,
	}, nil
}

// NewIMU returns a new IMU.
func NewIMU(
	ctx context.Context,
	deps resource.Dependencies,
	imuName string,
	logger golog.Logger,
) (IMU, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::sensors::NewIMU")
	defer span.End()
	if imuName == "" {
		logger.Info("no movement sensor configured, proceeding without IMU")
		return IMU{}, nil
	}
	newIMU, err := movementsensor.FromDependencies(deps, imuName)
	if err != nil {
		return IMU{}, errors.Wrapf(err, "error getting IMU movement sensor %v for slam service", imuName)
	}

	// A movement_sensor used as an IMU must support LinearAcceleration and AngularVelocity.
	properties, err := newIMU.Properties(ctx, make(map[string]interface{}))
	if err != nil {
		return IMU{}, errors.Wrapf(err, "error getting movement sensor properties %v for slam service", imuName)
	}
	if !(properties.LinearAccelerationSupported && properties.AngularVelocitySupported) {
		return IMU{}, errors.New("configuring IMU movement sensor error: " +
			"'movement_sensor' must support both LinearAcceleration and AngularVelocity")
	}

	return IMU{
		Name: imuName,
		imu:  newIMU,
	}, nil
}

// ValidateGetLidarData checks every sensorValidationIntervalSec if the provided lidar
// returned a valid timed readings every sensorValidationIntervalSec
// until either success or sensorValidationMaxTimeoutSec has elapsed.
// returns an error no valid reading was returned.
func ValidateGetLidarData(
	ctx context.Context,
	lidar TimedLidarSensor,
	sensorValidationMaxTimeout time.Duration,
	sensorValidationInterval time.Duration,
	logger golog.Logger,
) error {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::sensor::ValidateGetLidarData")
	defer span.End()

	startTime := time.Now().UTC()

	for {
		_, err := lidar.TimedLidarSensorReading(ctx)
		if err == nil {
			break
		}

		logger.Debugw("ValidateGetLidarData hit error: ", "error", err)
		if time.Since(startTime) >= sensorValidationMaxTimeout {
			return errors.Wrap(err, "ValidateGetLidarData timeout")
		}
		if !goutils.SelectContextOrWait(ctx, sensorValidationInterval) {
			return ctx.Err()
		}
	}

	return nil
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
	logger golog.Logger,
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
		if time.Since(startTime) >= sensorValidationMaxTimeout {
			return errors.Wrap(err, "ValidateGetIMUData timeout")
		}
		if !goutils.SelectContextOrWait(ctx, sensorValidationInterval) {
			return ctx.Err()
		}
	}

	return nil
}

// TimedLidarSensorReading returns data from the lidar sensor and the time the reading is from & whether it was a replay sensor or not.
func (lidar Lidar) TimedLidarSensorReading(ctx context.Context) (TimedLidarSensorReadingResponse, error) {
	live := true
	ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
	readingPc, err := lidar.lidar.NextPointCloud(ctxWithMetadata)
	if err != nil {
		msg := "NextPointCloud error"
		return TimedLidarSensorReadingResponse{}, errors.Wrap(err, msg)
	}
	readingTime := time.Now().UTC()

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary)
	if err != nil {
		msg := "ToPCD error"
		return TimedLidarSensorReadingResponse{}, errors.Wrap(err, msg)
	}

	timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
	if ok {
		live = false
		readingTime, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
		if err != nil {
			return TimedLidarSensorReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
		}
	}
	return TimedLidarSensorReadingResponse{Reading: buf.Bytes(), ReadingTime: readingTime, Replay: !live}, nil
}

// TimedIMUSensorReading returns data from the IMU movement sensor and the time the reading is from.
// IMU Sensors currently do not support replay capabilities.
func (imu IMU) TimedIMUSensorReading(ctx context.Context) (TimedIMUSensorReadingResponse, error) {
	live := true

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
				linAcc, err = imu.imu.LinearAcceleration(ctxWithMetadata, make(map[string]interface{}))
				if err != nil {
					msg := "LinearAcceleration error"
					return TimedIMUSensorReadingResponse{}, errors.Wrap(err, msg)
				}
				timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
				if ok {
					live = live && false
					timeLinearAcc, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
					if err != nil {
						return TimedIMUSensorReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
					}
				}
			}

			if timeAngularVel == defaultTime || timeAngularVel.Sub(timeLinearAcc).Milliseconds() < 0 {
				ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
				angVel, err = imu.imu.AngularVelocity(ctxWithMetadata, make(map[string]interface{}))
				if err != nil {
					msg := "AngularVelocity error"
					return TimedIMUSensorReadingResponse{}, errors.Wrap(err, msg)
				}
				timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
				if ok {
					live = live && false
					timeAngularVel, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
					if err != nil {
						return TimedIMUSensorReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
					}
				}
			}
			if live {
				return TimedIMUSensorReadingResponse{
					LinearAcceleration: linAcc,
					AngularVelocity: spatialmath.AngularVelocity{
						X: rdkutils.DegToRad(angVel.X),
						Y: rdkutils.DegToRad(angVel.Y),
						Z: rdkutils.DegToRad(angVel.Z),
					},
					ReadingTime: time.Now().UTC(),
					Replay:      false,
				}, nil
			} else if math.Abs(float64(timeAngularVel.Sub(timeLinearAcc).Milliseconds())) < replayTimeTolerance {
				return TimedIMUSensorReadingResponse{
					LinearAcceleration: linAcc,
					AngularVelocity: spatialmath.AngularVelocity{
						X: rdkutils.DegToRad(angVel.X),
						Y: rdkutils.DegToRad(angVel.Y),
						Z: rdkutils.DegToRad(angVel.Z),
					},
					ReadingTime: timeLinearAcc.Add(timeLinearAcc.Sub(timeAngularVel) / 2),
					Replay:      true,
				}, nil
			}
		}
	}
}
