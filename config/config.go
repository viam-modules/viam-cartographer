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
func (config *Config) Validate(path string) (map[string]string, error) {
	_, ok := config.Camera["name"]
	if !ok {
		return nil, utils.NewConfigValidationError(path, errCameraMustHaveName)
	}
	data_freq_hz, ok := config.Camera["data_freq_hz"]
	if ok {
		data_freq_hz, err := strconv.Atoi(data_freq_hz)
		if err != nil {
			return nil, errors.New("data_freq_hz must only contain digits")
		}
		if data_freq_hz < 0 {
			return nil, errors.New("cannot specify data_freq_hz less than zero")
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

	deps := config.Camera

	return deps, nil
}

// GetOptionalParameters sets any unset optional config parameters to the values passed to this function,
// and returns them.
func GetOptionalParameters(config *Config,
	defaultDataFreqHz, defaultMapRateSec int, logger golog.Logger,
) (int, int) {
	dataFreqHz := defaultDataFreqHz
	dataFreqHzIn, ok := config.Camera["data_freq_hz"]
	if !ok {
		logger.Debugf("no data_freq_hz given, setting to default value of %d", defaultDataFreqHz)
	} else {
		dataFreqHz, _ = strconv.Atoi(dataFreqHzIn)
	}

	mapRateSec := 0
	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_sec given, setting to default value of %d", defaultMapRateSec)
		mapRateSec = defaultMapRateSec
	} else {
		mapRateSec = *config.MapRateSec
	}

	return dataFreqHz, mapRateSec
}
