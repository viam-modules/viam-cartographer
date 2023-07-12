// Package imu implements the IMU sensor.
package imu

import (
	"context"
	"path/filepath"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils/contextutils"

	"github.com/viamrobotics/viam-cartographer/dataprocess"
	"github.com/viamrobotics/viam-cartographer/sensors/utils"
)

// Lidar represents a LIDAR sensor.
type IMU struct {
	Name string
	imu  movementsensor.MovementSensor
}

// New creates a new IMU sensor based on the sensor definition and the service config.
func New(deps resource.Dependencies, sensors []string, sensorIndex int) (IMU, error) {
	name, err := utils.GetName(sensors, sensorIndex)
	if err != nil {
		return IMU{}, err
	}

	newIMU, err := movementsensor.FromDependencies(deps, name)
	if err != nil {
		return IMU{}, errors.Wrapf(err, "error getting IMU sensor %v for slam service", name)
	}

	return IMU{
		Name: name,
		imu:  newIMU,
	}, nil
}

// GetLinearAcceleration returns linear acceleration from the IMU sensor.
func (imu IMU) GetLinearAcceleration(ctx context.Context) (r3.Vector, error) {
	return imu.imu.LinearAcceleration(ctx, make(map[string]interface{}))

}

// GetAngularVelocity returns angular velocity from the IMU sensor.
func (imu IMU) GetAngularVelocity(ctx context.Context) (spatialmath.AngularVelocity, error) {
	return imu.imu.AngularVelocity(ctx, make(map[string]interface{}))

}

const (
	opTimeoutErrorMessage = "bad scan: OpTimeout"
)

// GetAndSaveLidarData gets the pointcloud from the lidar and saves it to the provided data directory.
// On success, it returns the absolute filepath where the data was saved, along with any error
// associated with the data saving.
func GetAndSaveIMUData(ctx context.Context, dataDirectory string, imu IMU, logger golog.Logger) (string, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::internal::imu::GetAndSaveIMUData")
	defer span.End()

	ctx, md := contextutils.ContextWithMetadata(ctx)

	linearAcceleration, err := imu.GetLinearAcceleration(ctx)
	if err != nil {
		if err.Error() == opTimeoutErrorMessage {
			logger.Warnw("Skipping this scan due to error", "error", err)
			return "", nil
		}
		return "", err
	}

	angularVelocity, err := imu.GetAngularVelocity(ctx)
	if err != nil {
		if err.Error() == opTimeoutErrorMessage {
			logger.Warnw("Skipping this scan due to error", "error", err)
			return "", nil
		}
		return "", err
	}

	// If the server provided timestamps correlated with the point cloud, extract the time
	// requested from the metadata and use that instead of the current time.
	timeReq := time.Now().UTC()
	timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
	if ok {
		timeReq, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
		if err != nil {
			logger.Warnw("couldn't parse time", "error", err)
			return "", err
		}
	}

	dataDir := filepath.Join(dataDirectory, "data")
	filename := dataprocess.CreateTimestampFilename(dataDir, imu.Name, ".json", timeReq)
	return filename, dataprocess.WriteJSONToFile(linearAcceleration, angularVelocity, filename)
}
