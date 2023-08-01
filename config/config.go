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
	UseNewConfig  bool              `json:"use_new_config"`
	Sensors       []string          `json:"sensors"`
	DataRateMsec  int               `json:"data_rate_msec"`
}

var (
	errCameraMustHaveName    = errors.New("\"camera[name]\" is required")
	errSensorsMustNotBeEmpty = errors.New("\"sensors\" must not be empty")
)

// Validate creates the list of implicit dependencies.
func (config *Config) Validate(path string) ([]string, error) {
	cameraName := ""
	if config.UseNewConfig {
		var ok bool
		cameraName, ok = config.Camera["name"]
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
	} else {
		if config.Sensors == nil || len(config.Sensors) < 1 {
			return nil, utils.NewConfigValidationError(path, errors.New("\"sensors\" must not be empty"))
		}
		cameraName = config.Sensors[0]

		if config.DataRateMsec < 0 {
			return nil, errors.New("cannot specify data_rate_msec less than zero")
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
func GetOptionalParameters(config *Config, defaultLidarDataRateMsec, defaultMapRateSec int, logger golog.Logger,
) (int, int) {
	lidarDataRateMsec := defaultLidarDataRateMsec

	// feature flag for new config
	if config.UseNewConfig {
		strCameraDataFreqHz, ok := config.Camera["data_frequency_hz"]
		if !ok {
			logger.Debugf("config did not provide camera[data_frequency_hz], setting to default value of %d", 1000/defaultLidarDataRateMsec)
		} else {
			lidarDataFreqHz, _ := strconv.Atoi(strCameraDataFreqHz)
			lidarDataRateMsec = 1000 / lidarDataFreqHz
		}
	} else {
		lidarDataRateMsec = config.DataRateMsec
		if config.DataRateMsec == 0 {
			lidarDataRateMsec = defaultLidarDataRateMsec
			logger.Debugf("no data_rate_msec given, setting to default value of %d", defaultLidarDataRateMsec)
		}
	}

	mapRateSec := 0
	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_sec given, setting to default value of %d", defaultMapRateSec)
		mapRateSec = defaultMapRateSec
	} else {
		mapRateSec = *config.MapRateSec
	}

	return lidarDataRateMsec, mapRateSec
}
