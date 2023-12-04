package sensors

import (
	"bytes"
	"context"
	"time"

	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils/contextutils"
)

// TimedLidar describes a sensor that reports the time the reading is from & whether or not it is
// rom a replay sensor.
type TimedLidar interface {
	Name() string
	DataFrequencyHz() int
	TimedLidarReading(ctx context.Context) (TimedLidarReadingResponse, error)
}

// TimedLidarReadingResponse represents a lidar reading with a time & allows the caller
// to know if the reading is from a replay camera.
type TimedLidarReadingResponse struct {
	Reading        []byte
	ReadingTime    time.Time
	IsReplaySensor bool
}

// Lidar represents a LIDAR sensor.
type Lidar struct {
	name            string
	dataFrequencyHz int
	Lidar           camera.Camera
}

// Name returns the name of the lidar.
func (lidar Lidar) Name() string {
	return lidar.name
}

// DataFrequencyHz returns the data rate in ms of the lidar.
func (lidar Lidar) DataFrequencyHz() int {
	return lidar.dataFrequencyHz
}

// TimedLidarReading returns data from the lidar and the time the reading is from & whether
// it was a replay sensor or not.
func (lidar Lidar) TimedLidarReading(ctx context.Context) (TimedLidarReadingResponse, error) {
	replay := false

	ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
	readingPc, err := lidar.Lidar.NextPointCloud(ctxWithMetadata)
	if err != nil {
		return TimedLidarReadingResponse{}, errors.Wrap(err, "NextPointCloud error")
	}
	readingTime := time.Now().UTC()

	buf := new(bytes.Buffer)
	if err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary); err != nil {
		return TimedLidarReadingResponse{}, errors.Wrap(err, "ToPCD error")
	}

	if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
		replay = true
		if readingTime, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0]); err != nil {
			return TimedLidarReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
		}
	}
	return TimedLidarReadingResponse{Reading: buf.Bytes(), ReadingTime: readingTime, IsReplaySensor: replay}, nil
}

// NewLidar returns a new Lidar.
func NewLidar(
	ctx context.Context,
	deps resource.Dependencies,
	cameraName string,
	dataFrequencyHz int,
	logger logging.Logger,
) (TimedLidar, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::sensors::NewLidar")
	defer span.End()
	lidar, err := camera.FromDependencies(deps, cameraName)
	if err != nil {
		return Lidar{}, errors.Wrapf(err, "error getting lidar camera %v for slam service", cameraName)
	}

	// If there is a camera provided in the 'camera' field, we enforce that it supports PCD.
	properties, err := lidar.Properties(ctx)
	if err != nil {
		return Lidar{}, errors.Wrapf(err, "error getting lidar camera properties %v for slam service", cameraName)
	}

	if !properties.SupportsPCD {
		return Lidar{}, errors.New("configuring lidar camera error: " +
			"'camera' must support PCD")
	}

	return Lidar{
		name:            cameraName,
		dataFrequencyHz: dataFrequencyHz,
		Lidar:           lidar,
	}, nil
}
