// Package sensors defines interfaces for sensors used by viam cartographer
package sensors

import (
	"bytes"
	"context"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils/contextutils"
	goutils "go.viam.com/utils"
)

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

// TimedSensorReadingResponse represents a sensor reading with a time & allows the caller to know if the reading is from a replay sensor.
type TimedSensorReadingResponse struct {
	Reading     []byte
	ReadingTime time.Time
	Replay      bool
}

// TimedSensor desribes a sensor that reports the time the reading is from & whether or not it is from a replay sensor.
type TimedSensor interface {
	TimedLidarSensorReading(ctx context.Context) (TimedSensorReadingResponse, error)
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

	// https://viam.atlassian.net/browse/RSDK-4306
	// To be implemented once replay camera supports Properties
	// // If there is a camera provided in the 'camera' field, we enforce that it supports PCD.
	// properties, err := newLidar.Properties(ctx)
	// if err != nil {
	// 	return Lidar{}, errors.Wrapf(err, "error getting lidar camera properties %v for slam service", cameraName)
	// }

	// if !properties.SupportsPCD {
	// 	return Lidar{}, errors.New("configuring lidar camera error: " +
	// 		"'camera' must support PCD")
	// }

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
	newIMU, err := movementsensor.FromDependencies(deps, imuName)
	if err != nil {
		return IMU{}, errors.Wrapf(err, "error getting imu movement sensor %v for slam service", imuName)
	}

	// A movement_sensor used as an IMU must support LinearAcceleration and Angular Velocity.
	properties, err := newIMU.Properties(ctx, make(map[string]interface{}))
	if err != nil {
		return IMU{}, errors.Wrapf(err, "error getting movement sensor properties %v for slam service", imuName)
	}
	if !(properties.LinearAccelerationSupported && properties.AngularVelocitySupported) {
		return IMU{}, errors.New("configuring imu movement sensor error: " +
			"'movement_sensor' must support both LinearAcceleration and AngularVelocity")
	}

	return IMU{
		Name: imuName,
		imu:  newIMU,
	}, nil
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
		_, err := sensor.TimedLidarSensorReading(ctx)
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

// TimedLidarSensorReading returns data from the lidar sensor and the time the reading is from & whether it was a replay sensor or not.
func (lidar Lidar) TimedLidarSensorReading(ctx context.Context) (TimedSensorReadingResponse, error) {
	replay := false
	ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
	readingPc, err := lidar.lidar.NextPointCloud(ctxWithMetadata)
	if err != nil {
		msg := "NextPointCloud error"
		return TimedSensorReadingResponse{}, errors.Wrap(err, msg)
	}
	readingTime := time.Now().UTC()

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary)
	if err != nil {
		msg := "ToPCD error"
		return TimedSensorReadingResponse{}, errors.Wrap(err, msg)
	}

	timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
	if ok {
		replay = true
		readingTime, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
		if err != nil {
			msg := "replay sensor timestamp parse RFC3339Nano error"
			return TimedSensorReadingResponse{}, errors.Wrap(err, msg)
		}
	}
	return TimedSensorReadingResponse{Reading: buf.Bytes(), ReadingTime: readingTime, Replay: replay}, nil
}

// TimedIMUSensorReading returns data from the IMU movement sensor and the time the reading is from.
// IMU Sensors currently do not support replay capabilities.
func (imu IMU) TimedIMUSensorReading(ctx context.Context) (TimedSensorReadingResponse, error) {
	replay := false
	ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
	linAcc, err := imu.imu.LinearAcceleration(ctxWithMetadata, make(map[string]interface{}))
	if err != nil {
		msg := "LinearAcceleration error"
		return TimedSensorReadingResponse{}, errors.Wrap(err, msg)
	}
	angVel, err := imu.imu.AngularVelocity(ctxWithMetadata, make(map[string]interface{}))
	if err != nil {
		msg := "AngularVelocity error"
		return TimedSensorReadingResponse{}, errors.Wrap(err, msg)
	}
	imuData := [linAcc, angVel]
	readingTime := time.Now().UTC()

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary)
	if err != nil {
		msg := "ToPCD error"
		return TimedSensorReadingResponse{}, errors.Wrap(err, msg)
	}

	timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
	if ok {
		replay = true
		readingTime, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
		if err != nil {
			msg := "replay sensor timestamp parse RFC3339Nano error"
			return TimedSensorReadingResponse{}, errors.Wrap(err, msg)
		}
	}
	return TimedSensorReadingResponse{Reading: buf.Bytes(), ReadingTime: readingTime, Replay: replay}, nil
}
