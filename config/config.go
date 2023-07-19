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
	Sensors       []string          `json:"sensors"`
	ConfigParams  map[string]string `json:"config_params"`
	DataDirectory string            `json:"data_dir"`
	DataRateMsec  int               `json:"data_rate_msec"`
	MapRateSec    *int              `json:"map_rate_sec"`
}

var errSensorsMustNotBeEmpty = errors.New("\"sensors\" must not be empty")

// Validate creates the list of implicit dependencies.
func (config *Config) Validate(path string) ([]string, error) {
	if config.Sensors == nil || len(config.Sensors) < 1 {
		return nil, utils.NewConfigValidationError(path, errSensorsMustNotBeEmpty)
	}

	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	if config.DataDirectory == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "data_dir")
	}

	if config.DataRateMsec < 0 {
		return nil, errors.New("cannot specify data_rate_msec less than zero")
	}

	if config.MapRateSec != nil && *config.MapRateSec < 0 {
		return nil, errors.New("cannot specify map_rate_sec less than zero")
	}

	deps := config.Sensors

	return deps, nil
}

// GetOptionalParameters sets any unset optional config parameters to the values passed to this function,
// and returns them.
func GetOptionalParameters(config *Config,
	defaultDataRateMsec, defaultMapRateSec int, logger golog.Logger,
) (int, int) {
	dataRateMsec := config.DataRateMsec
	if config.DataRateMsec == 0 {
		dataRateMsec = defaultDataRateMsec
		logger.Debugf("no data_rate_msec given, setting to default value of %d", defaultDataRateMsec)
	}

	mapRateSec := 0
	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_sec given, setting to default value of %d", defaultMapRateSec)
		mapRateSec = defaultMapRateSec
	} else {
		mapRateSec = *config.MapRateSec
	}

	return dataRateMsec, mapRateSec
}
