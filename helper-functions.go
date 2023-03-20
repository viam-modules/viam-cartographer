// Package viamcartographer implements simultaneous localization and mapping
// This is an Experimental package
package viamcartographer

import (
	"context"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/registry"
	slamConfig "go.viam.com/slam/config"
	"go.viam.com/slam/dataprocess"
	goutils "go.viam.com/utils"
)

// runtimeServiceValidation ensures the service's data processing and saving is valid for the subAlgo and
// cameras given.
func runtimeServiceValidation(
	ctx context.Context,
	cams []camera.Camera,
	cartoSvc *cartographerService,
) error {
	if !cartoSvc.useLiveData {
		return nil
	}

	var err error
	var path string
	paths := make([]string, 0, 1)
	startTime := time.Now()

	for {
		path, err = cartoSvc.getAndSaveData(ctx, cams)
		paths = append(paths, path)

		if err == nil {
			break
		}

		// This takes about 5 seconds, so the timeout should be sufficient.
		if time.Since(startTime) >= time.Duration(cameraValidationMaxTimeoutSec)*time.Second {
			return errors.Wrap(err, "error getting data from sensor")
		}
		if !goutils.SelectContextOrWait(ctx, cameraValidationIntervalSec*time.Second) {
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

// configureCameras will check the config to see if any cameras are desired and if so, grab the cameras from
// the robot. We assume there is at most one lidar camera.
func configureCameras(
	svcConfig *slamConfig.AttrConfig,
	deps registry.Dependencies,
) (string, []camera.Camera, error) {
	if len(svcConfig.Sensors) == 0 {
		return "", nil, nil
	}
	if len(svcConfig.Sensors) != 1 {
		return "", nil, errors.Errorf("'sensors' must contain only one lidar camera, but is 'sensors: [%v]'",
			strings.Join(svcConfig.Sensors, ", "))
	}
	cams := make([]camera.Camera, 0, len(svcConfig.Sensors))
	// The first camera is expected to be LIDAR.
	primarySensorName := svcConfig.Sensors[0]
	cam, err := camera.FromDependencies(deps, primarySensorName)
	if err != nil {
		return "", nil, errors.Wrapf(err, "error getting camera %v for slam service", primarySensorName)
	}
	cams = append(cams, cam)
	return primarySensorName, cams, nil
}

// SetCameraValidationMaxTimeoutSecForTesting sets cameraValidationMaxTimeoutSec for testing.
func SetCameraValidationMaxTimeoutSecForTesting(val int) {
	cameraValidationMaxTimeoutSec = val
}

// SetDialMaxTimeoutSecForTesting sets dialMaxTimeoutSec for testing.
func SetDialMaxTimeoutSecForTesting(val int) {
	dialMaxTimeoutSec = val
}

// Creates a file for camera data with the specified sensor name and timestamp written into the filename.
func createTimestampFilename(dataDirectory, primarySensorName, fileType string) string {
	timeStamp := time.Now()
	dataDir := filepath.Join(dataDirectory, "data")
	return dataprocess.CreateTimestampFilename(dataDir, primarySensorName, fileType, timeStamp)
}
