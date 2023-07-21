// Package config implements functions to assist with attribute evaluation in the SLAM service.
package config

import (
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
	DataFreqHz    int               `json:"data_freq_hz"`
	MapRateSec    *int              `json:"map_rate_sec"`
}

var errCameraMustNotBeEmpty = errors.New("\"camera\" must not be empty")

// Validate creates the list of implicit dependencies.
func (config *Config) Validate(path string) (map[string]string, error) {
	if config.Camera == nil || len(config.Camera) < 1 {
		return nil, utils.NewConfigValidationError(path, errCameraMustNotBeEmpty)
	}

	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	if config.DataDirectory == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "data_dir")
	}

	if config.DataFreqHz < 0 {
		return nil, errors.New("cannot specify data_freq_hz less than zero")
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
	dataFreqHz := config.DataFreqHz
	if config.DataFreqHz == 0 {
		dataFreqHz = defaultDataFreqHz
		logger.Debugf("no data_freq_hz given, setting to default value of %d", defaultDataFreqHz)
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
