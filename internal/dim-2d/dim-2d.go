// Package dim2d implements the 2D sub algorithm
package dim2d

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	"go.viam.com/rdk/registry"
	"go.viam.com/slam/dataprocess"
	"go.viam.com/slam/sensors/lidar"
	goutils "go.viam.com/utils"
)

const (
	opTimeoutErrorMessage = "bad scan: OpTimeout"
)

// NewLidar returns a new lidar.Lidar for the 2D cartographer mode.
func NewLidar(
	ctx context.Context,
	deps registry.Dependencies,
	sensors []string,
	logger golog.Logger,
) (lidar.Lidar, error) {
	ctx, span := trace.StartSpan(ctx, "dim2d::NewLidar")
	defer span.End()

	lidar, err := getLidar(ctx, sensors, deps, logger)
	if err != nil {
		return lidar, errors.Wrap(err, "configuring lidar camera error")
	}

	return lidar, nil
}

// getLidar will check the config to see if any lidar camera is provided. If yes,
// grab the cameras from the robot. We assume there is at most one lidar camera.
func getLidar(
	ctx context.Context,
	sensors []string,
	deps registry.Dependencies,
	logger golog.Logger,
) (lidar.Lidar, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::internal::dim2d::getLidar")
	defer span.End()

	// An empty `sensors: []` array is allowed in offline mode.
	if len(sensors) == 0 {
		logger.Debug("no sensor provided in 'sensors' config parameter")
		return lidar.Lidar{}, nil
	}
	// If there is a sensor provided in the 'sensors' array, we enforce that only one
	// sensor has to be provided.
	if len(sensors) != 1 {
		return lidar.Lidar{}, errors.Errorf("'sensors' must contain only one lidar camera, but is 'sensors: [%v]'",
			strings.Join(sensors, ", "))
	}
	return lidar.New(ctx, deps, sensors, lidar2DIndex)
}

// ValidateGetAndSaveData makes sure that the provided sensor is actually a lidar and can
// return pointclouds. It also ensures that saving the data to files works as intended.
func ValidateGetAndSaveData(
	ctx context.Context,
	dataDirectory string,
	lidar lidar.Lidar,
	sensorValidationMaxTimeoutSec int,
	sensorValidationIntervalSec int,
	logger golog.Logger,
) error {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::internal::dim2d::ValidateGetAndSaveData")
	defer span.End()

	var err error
	var path string
	paths := make([]string, 0, 1)
	startTime := time.Now()

	for {
		path, err = GetAndSaveData(ctx, dataDirectory, lidar, logger)
		paths = append(paths, path)

		if err == nil {
			break
		}

		// This takes about 5 seconds in real life, so a default timeout of
		// defaultSensorValidationMaxTimeoutSec = 30 as it is used for the Constructor in
		// viam-cartographer.go should be sufficient.
		if time.Since(startTime) >= time.Duration(sensorValidationMaxTimeoutSec)*time.Second {
			return errors.Wrap(err, "error getting data from sensor")
		}
		if !goutils.SelectContextOrWait(ctx, time.Duration(sensorValidationIntervalSec)*time.Second) {
			return ctx.Err()
		}
	}

	for _, path := range paths {
		if err := os.RemoveAll(path); err != nil {
			return errors.Wrap(err, "error removing generated file during validation")
		}
	}

	return nil
}

// GetAndSaveData gets the pointcloud from the lidar and saves it to the provided data directory.
// On success, it returns the absolute filepath where the data was saved, along with any error
// associated with the data saving.
func GetAndSaveData(ctx context.Context, dataDirectory string, lidar lidar.Lidar, logger golog.Logger) (string, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::internal::dim2d::GetAndSaveData")
	defer span.End()

	pointcloud, err := lidar.GetData(ctx)
	if err != nil {
		if err.Error() == opTimeoutErrorMessage {
			logger.Warnw("Skipping this scan due to error", "error", err)
			return "", nil
		}
		return "", err
	}

	filename := createTimestampFilename(dataDirectory, lidar.Name, ".pcd")
	return filename, dataprocess.WritePCDToFile(pointcloud, filename)
}

// Creates a file for camera data with the specified sensor name and timestamp written into the filename.
func createTimestampFilename(dataDirectory, primarySensorName, fileType string) string {
	timeStamp := time.Now()
	dataDir := filepath.Join(dataDirectory, "data")
	return dataprocess.CreateTimestampFilename(dataDir, primarySensorName, fileType, timeStamp)
}
