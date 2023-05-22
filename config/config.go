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

// DetermineDeleteProcessedData will determine the value of the deleteProcessData attribute
// based on the useLiveData and deleteData input parameters.
func DetermineDeleteProcessedData(logger golog.Logger, deleteData *bool, useLiveData bool) bool {
	var deleteProcessedData bool
	if deleteData == nil {
		deleteProcessedData = useLiveData
	} else {
		deleteProcessedData = *deleteData
		if !useLiveData && deleteProcessedData {
			logger.Debug("a value of true cannot be given for delete_processed_data when in offline mode, setting to false")
			deleteProcessedData = false
		}
	}
	return deleteProcessedData
}

// DetermineUseLiveData will determine the value of the useLiveData attribute
// based on the liveData input parameter and sensor list.
func DetermineUseLiveData(logger golog.Logger, liveData *bool, sensors []string) (bool, error) {
	if liveData == nil {
		return false, newError("use_live_data is a required input parameter")
	}
	useLiveData := *liveData
	if useLiveData && len(sensors) == 0 {
		return false, newError("sensors field cannot be empty when use_live_data is set to true")
	}
	return useLiveData, nil
}

// Config describes how to configure the SLAM service.
type Config struct {
	Sensors                 []string          `json:"sensors"`
	ConfigParams            map[string]string `json:"config_params"`
	DataDirectory           string            `json:"data_dir"`
	UseLiveData             *bool             `json:"use_live_data"`
	DataRateMsec            int               `json:"data_rate_msec"`
	MapRateSec              *int              `json:"map_rate_sec"`
	Port                    string            `json:"port"`
	DeleteProcessedData     *bool             `json:"delete_processed_data"`
	ModularizationV2Enabled *bool             `json:"modularization_v2_enabled"`
}

// Validate creates the list of implicit dependencies.
func (config *Config) Validate(path string) ([]string, error) {
	// get feature flag provided in config
	modularizationV2Enabled := false
	if config.ModularizationV2Enabled != nil {
		modularizationV2Enabled = *config.ModularizationV2Enabled
	}

	// require at least one sensor for full mod v2
	if modularizationV2Enabled == true {
		if config.Sensors == nil || len(config.Sensors) < 1 {
			return nil, utils.NewConfigValidationFieldRequiredError(path, "at least one sensor must be configured")
		}
	}

	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	if config.DataDirectory == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "data_dir")
	}

	// do not validate use_live_data if mod v2 is enabled
	if config.UseLiveData == nil && modularizationV2Enabled == false {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "use_live_data")
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
func GetOptionalParameters(config *Config, defaultPort string,
	defaultDataRateMsec, defaultMapRateSec int, logger golog.Logger,
) (string, int, int, bool, bool, bool, error) {
	modularizationV2Enabled := false
	if config.ModularizationV2Enabled == nil {
		logger.Debug("no modularization_v2_enabled given, continuing with modularization v1")
	} else {
		modularizationV2Enabled = *config.ModularizationV2Enabled
	}

	// do not validate port if mod v2 is enabled
	port := config.Port
	if config.Port == "" && modularizationV2Enabled == false {
		port = defaultPort
	}

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
	if mapRateSec == 0 {
		logger.Info("setting slam system to localization mode")
	}

	useLiveData, err := DetermineUseLiveData(logger, config.UseLiveData, config.Sensors)
	if err != nil {
		return "", 0, 0, false, false, false, err
	}

	// only validate deleteProcessedData if mod v2 is not enabled
	deleteProcessedData := false
	if modularizationV2Enabled == false {
		deleteProcessedData = DetermineDeleteProcessedData(logger, config.DeleteProcessedData, useLiveData)
	}

	return port, dataRateMsec, mapRateSec, useLiveData, deleteProcessedData, modularizationV2Enabled, nil
}
