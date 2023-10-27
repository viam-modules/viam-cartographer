package sensors

import (
	"bytes"
	"context"
	"strings"
	"time"

	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/camera/replaypcd"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils/contextutils"
	goutils "go.viam.com/utils"
)

// TimedLidarSensor describes a sensor that reports the time the reading is from & whether or not it is
// rom a replay sensor.
type TimedLidarSensor interface {
	Name() string
	DataFrequencyHz() int
	TimedLidarSensorReading(ctx context.Context) (TimedLidarSensorReadingResponse, error)
}

// TimedLidarSensorReadingResponse represents a lidar sensor reading with a time & allows the caller
// to know if the reading is from a replay camera sensor.
type TimedLidarSensorReadingResponse struct {
	Reading     []byte
	ReadingTime time.Time
	Replay      bool
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

// TimedLidarSensorReading returns data from the lidar sensor and the time the reading is from & whether
// it was a replay sensor or not.
func (lidar Lidar) TimedLidarSensorReading(ctx context.Context) (TimedLidarSensorReadingResponse, error) {
	replay := false

	ctxWithMetadata, md := contextutils.ContextWithMetadata(ctx)
	readingPc, err := lidar.Lidar.NextPointCloud(ctxWithMetadata)
	if err != nil {
		msg := "NextPointCloud error"
		return TimedLidarSensorReadingResponse{}, errors.Wrap(err, msg)
	}
	readingTime := time.Now().UTC()

	buf := new(bytes.Buffer)
	if err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary); err != nil {
		msg := "ToPCD error"
		return TimedLidarSensorReadingResponse{}, errors.Wrap(err, msg)
	}

	if timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]; ok {
		replay = true
		if readingTime, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0]); err != nil {
			return TimedLidarSensorReadingResponse{}, errors.Wrap(err, replayTimestampErrorMessage)
		}
	}
	return TimedLidarSensorReadingResponse{Reading: buf.Bytes(), ReadingTime: readingTime, Replay: replay}, nil
}

// NewLidar returns a new Lidar.
func NewLidar(
	ctx context.Context,
	deps resource.Dependencies,
	cameraName string,
	dataFrequencyHz int,
	logger logging.Logger,
) (TimedLidarSensor, error) {
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

// ValidateGetLidarData checks every sensorValidationIntervalSec if the provided lidar
// returned a valid timed readings every sensorValidationIntervalSec
// until either success or sensorValidationMaxTimeoutSec has elapsed.
// Returns an error no valid reading was returned.
func ValidateGetLidarData(
	ctx context.Context,
	lidar TimedLidarSensor,
	sensorValidationMaxTimeout time.Duration,
	sensorValidationInterval time.Duration,
	logger logging.Logger,
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
		// if the sensor is a replay camera with no data ready, allow validation to pass
		// offline mode will stop the mapping session if the sensor still has no data,
		// while online mode will continue mapping once data is found by the replay sensor
		if strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error()) {
			break
		}
		if time.Since(startTime) >= sensorValidationMaxTimeout {
			return errors.Wrap(err, "ValidateGetLidarData timeout")
		}
		if !goutils.SelectContextOrWait(ctx, sensorValidationInterval) {
			return ctx.Err()
		}
	}

	return nil
}
