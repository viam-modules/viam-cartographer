// Package lidar implements the Lidar sensor.
package lidar

import (
	"bytes"
	"context"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils/contextutils"

	"github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/utils"
)

// Lidar represents a LIDAR sensor.
type Lidar struct {
	Name  string
	lidar camera.Camera
}

// New creates a new Lidar sensor based on the sensor definition and the service config.
func New(deps resource.Dependencies, sensors []string, sensorIndex int) (Lidar, error) {
	name, err := utils.GetName(sensors, sensorIndex)
	if err != nil {
		return Lidar{}, err
	}

	newLidar, err := camera.FromDependencies(deps, name)
	if err != nil {
		return Lidar{}, errors.Wrapf(err, "error getting lidar camera %v for slam service", name)
	}

	return Lidar{
		Name:  name,
		lidar: newLidar,
	}, nil
}

// GetData returns data from the lidar sensor.
// deprecated.
func (lidar Lidar) GetData(ctx context.Context) (pointcloud.PointCloud, error) {
	return lidar.lidar.NextPointCloud(ctx)
}

// TimedSensorReading returns data from the lidar sensor and the time the reading is from & whether it was a replay sensor or not.
func (lidar Lidar) TimedSensorReading(ctx context.Context) (sensors.TimedSensorReadingResponse, error) {
	replay := false
	ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
	readingPc, err := lidar.lidar.NextPointCloud(ctxWithMetadata)
	if err != nil {
		msg := "NextPointCloud error"
		return sensors.TimedSensorReadingResponse{}, errors.Wrap(err, msg)
	}
	readingTime := time.Now().UTC()

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary)
	if err != nil {
		msg := "ToPCD error"
		return sensors.TimedSensorReadingResponse{}, errors.Wrap(err, msg)
	}

	timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
	if ok {
		replay = true
		readingTime, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
		if err != nil {
			msg := "replay sensor timestamp parse RFC3339Nano error"
			return sensors.TimedSensorReadingResponse{}, errors.Wrap(err, msg)
		}
	}
	return sensors.TimedSensorReadingResponse{Reading: buf.Bytes(), ReadingTime: readingTime, Replay: replay}, nil
}
