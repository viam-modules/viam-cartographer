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
	Camera                map[string]string `json:"camera"`
	MovementSensor        map[string]string `json:"movement_sensor"`
	ConfigParams          map[string]string `json:"config_params"`
	DataDirectory         string            `json:"data_dir"`
	MapRateSec            *int              `json:"map_rate_sec"`
	IMUIntegrationEnabled bool              `json:"imu_integration_enabled"`
	Sensors               []string          `json:"sensors"`
	DataRateMsec          *int              `json:"data_rate_msec"`

	CloudStoryEnabled bool   `json:"cloud_story_enabled"`
	ExistingMap       string `json:"existing_map"`
	EnableMapping     bool   `json:"enable_mapping"`
	RunSlam           bool   `json:"run_slam"`
}

var (
	errCameraMustHaveName    = errors.New("\"camera[name]\" is required")
	errSensorsMustNotBeEmpty = errors.New("\"sensors\" must not be empty")
)

// Validate creates the list of implicit dependencies.
func (config *Config) Validate(path string) ([]string, error) {
	cameraName := ""

	if config.IMUIntegrationEnabled {
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
			return nil, utils.NewConfigValidationError(path, errSensorsMustNotBeEmpty)
		}
		cameraName = config.Sensors[0]

		if config.DataRateMsec != nil && *config.DataRateMsec < 0 {
			return nil, errors.New("cannot specify data_rate_msec less than zero")
		}
	}

	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	if config.CloudStoryEnabled {
		if config.ExistingMap == "" {
			return nil, utils.NewConfigValidationFieldRequiredError(path, "existing_map")
		}
	} else {
		if config.DataDirectory == "" {
			return nil, utils.NewConfigValidationFieldRequiredError(path, "data_dir")
		}

		if config.MapRateSec != nil && *config.MapRateSec < 0 {
			return nil, errors.New("cannot specify map_rate_sec less than zero")
		}
	}

	deps := []string{cameraName}

	return deps, nil
}

// GetOptionalParameters sets any unset optional config parameters to the values passed to this function,
// and returns them.
func GetOptionalParameters(config *Config, defaultLidarDataRateMsec, defaultIMUDataRateMsec, defaultMapRateSec int, logger golog.Logger,
) (int, string, int, int, error) {
	lidarDataRateMsec := defaultLidarDataRateMsec
	imuName := ""
	imuDataRateMsec := defaultIMUDataRateMsec

	// feature flag for new config
	if config.IMUIntegrationEnabled {
		strCameraDataFreqHz, ok := config.Camera["data_frequency_hz"]
		if !ok {
			logger.Debugf("config did not provide camera[data_frequency_hz], setting to default value of %d", 1000/defaultLidarDataRateMsec)
		} else {
			lidarDataFreqHz, err := strconv.Atoi(strCameraDataFreqHz)
			if err != nil {
				return 0, "", 0, 0, newError("camera[data_frequency_hz] must only contain digits")
			}
			lidarDataRateMsec = 1000 / lidarDataFreqHz
		}
		exists := false
		imuName, exists = config.MovementSensor["name"]
		if exists {
			strMovementSensorDataFreqHz, ok := config.MovementSensor["data_frequency_hz"]
			if !ok {
				logger.Debugf("config did not provide movement_sensor[data_frequency_hz], setting to default value of %d", 1000/defaultIMUDataRateMsec)
			} else {
				imuDataFreqHz, err := strconv.Atoi(strMovementSensorDataFreqHz)
				if err != nil {
					return 0, "", 0, 0, newError("movement_sensor[data_frequency_hz] must only contain digits")
				}
				imuDataRateMsec = 1000 / imuDataFreqHz
			}
		}
	} else {
		if config.DataRateMsec == nil {
			lidarDataRateMsec = defaultLidarDataRateMsec
			logger.Debugf("no data_rate_msec given, setting to default value of %d", defaultLidarDataRateMsec)
		} else {
			lidarDataRateMsec = *config.DataRateMsec
		}
	}

	mapRateSec := 0
	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_sec given, setting to default value of %d", defaultMapRateSec)
		mapRateSec = defaultMapRateSec
	} else {
		mapRateSec = *config.MapRateSec
	}

	return lidarDataRateMsec, imuName, imuDataRateMsec, mapRateSec, nil
}
