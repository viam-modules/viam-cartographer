// Package imu implements the IMU sensor.
package imu

import (
	"context"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

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

// GetAngularVelocity returns angular velocity from the IMU sensor.
func (imu IMU) GetAngularVelocity(ctx context.Context) (spatialmath.AngularVelocity, error) {
	return imu.imu.AngularVelocity(ctx, make(map[string]interface{}))

}

// GetLinearAcceleration returns linear acceleration from the IMU sensor.
func (imu IMU) GetLinearAcceleration(ctx context.Context) (r3.Vector, error) {
	return imu.imu.LinearAcceleration(ctx, make(map[string]interface{}))

}
