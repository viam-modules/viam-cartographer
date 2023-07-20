// Package sensors defines interfaces for sensors used by viam cartographer
package sensors

import (
	"bytes"
	"context"
	"strings"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/utils/contextutils"
	goutils "go.viam.com/utils"
)

const (
	// The Lidar is expected to be located at the first
	// index in the provided `sensors` array in the slam
	// service configuration.
	lidarIndex = 0
)

// Lidar represents a LIDAR sensor.
type Lidar struct {
	Name  string
	lidar camera.Camera
}

// TimedSensorReadingResponse represents a sensor reading with a time & allows the caller to know if the reading is from a replay sensor.
type TimedSensorReadingResponse struct {
	Reading     []byte
	ReadingTime time.Time
	Replay      bool
}

// TimedSensor desribes a sensor that reports the time the reading is from & whether or not it is from a replay sensor.
type TimedSensor interface {
	TimedSensorReading(ctx context.Context) (TimedSensorReadingResponse, error)
}

// NewLidar returns a new Lidar.
func NewLidar(
	ctx context.Context,
	deps resource.Dependencies,
	sensors []string,
	logger golog.Logger,
) (Lidar, error) {
	_, span := trace.StartSpan(ctx, "viamcartographer::sensors::NewLidar")
	defer span.End()

	// An empty `sensors: []` array is allowed in offline mode.
	if len(sensors) == 0 {
		logger.Debug("no sensor provided in 'sensors' config parameter")
		return Lidar{}, nil
	}
	// If there is a sensor provided in the 'sensors' array, we enforce that only one
	// sensor has to be provided.
	if len(sensors) != 1 {
		return Lidar{}, errors.Errorf("configuring lidar camera error: "+
			"'sensors' must contain only one lidar camera, but is 'sensors: [%v]'",
			strings.Join(sensors, ", "))
	}

	name, err := getName(sensors, lidarIndex)
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
		_, err := sensor.TimedSensorReading(ctx)
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

// TimedSensorReading returns data from the lidar sensor and the time the reading is from & whether it was a replay sensor or not.
func (lidar Lidar) TimedSensorReading(ctx context.Context) (TimedSensorReadingResponse, error) {
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

// getName returns the name of the sensor based on its index in the sensor array.
func getName(sensors []string, index int) (string, error) {
	if index < 0 || index >= len(sensors) {
		return "", errors.New("index out of bounds")
	}
	return sensors[index], nil
}
