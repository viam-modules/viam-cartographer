// Package config implements functions to assist with attribute evaluation in the SLAM service.
package config

import (
	"strconv"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.viam.com/utils"
)

// newError returns an error specific to a failure in the SLAM config.
func newError(configError string) error {
	return errors.Errorf("SLAM Service configuration error: %s", configError)
}

// Config describes how to configure the SLAM service.
type Config struct {
	Camera        map[string]string `json:"camera"`
	ConfigParams  map[string]string `json:"config_params"`
	DataDirectory string            `json:"data_dir"`
	MapRateSec    *int              `json:"map_rate_sec"`
}

var errCameraMustHaveName = errors.New("\"camera[name]\" is required")

// Validate creates the list of implicit dependencies.
func (config *Config) Validate(path string) ([]string, error) {
	cameraName, ok := config.Camera["name"]
	if !ok {
		return nil, utils.NewConfigValidationError(path, errCameraMustHaveName)
	}
	dataFreqHz, ok := config.Camera["data_frequency_hz"]
	if ok {
		dataFreqHz, err := strconv.Atoi(dataFreqHz)
		if err != nil {
			return nil, errors.New("camera[data_frequency_hz] must only contain digits")
		}
		if dataFreqHz < 0 {
			return nil, errors.New("cannot specify camera[data_frequency_hz] less than zero")
		}
	}

	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	if config.DataDirectory == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "data_dir")
	}

	if config.MapRateSec != nil && *config.MapRateSec < 0 {
		return nil, errors.New("cannot specify map_rate_sec less than zero")
	}

	deps := []string{cameraName}

	return deps, nil
}

// GetOptionalParameters sets any unset optional config parameters to the values passed to this function,
// and returns them.
func GetOptionalParameters(config *Config, defaultLidarDataRateMSec, defaultMapRateSec int, logger golog.Logger,
) (int, int, error) {
	lidarDataRateMSec := defaultLidarDataRateMSec
	strCameraDataFreqHz, ok := config.Camera["data_frequency_hz"]
	if !ok {
		logger.Debugf("problem retrieving lidar data frequency, setting to default value of %d", 1000/defaultLidarDataRateMSec)
	} else {
		lidarDataFreqHz, err := strconv.Atoi(strCameraDataFreqHz)
		if err != nil {
			return 0, 0, errors.New("camera[data_frequency_hz] must only contain digits")
		}
		lidarDataRateMSec = 1000 / lidarDataFreqHz
	}

	mapRateSec := 0
	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_sec given, setting to default value of %d", defaultMapRateSec)
		mapRateSec = defaultMapRateSec
	} else {
		mapRateSec = *config.MapRateSec
	}

	return lidarDataRateMSec, mapRateSec, nil
}
