// Package config implements functions to assist with attribute evaluation in the SLAM service.
package config

import (
	"strconv"
	"strings"

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
	Camera         map[string]string `json:"camera"`
	MovementSensor map[string]string `json:"movement_sensor"`
	ConfigParams   map[string]string `json:"config_params"`
	DataDirectory  string            `json:"data_dir"`
	MapRateSec     *int              `json:"map_rate_sec"`
	Sensors        []string          `json:"sensors"`
	DataRateMsec   *int              `json:"data_rate_msec"`

	CloudStoryEnabled bool   `json:"cloud_story_enabled"`
	ExistingMap       string `json:"existing_map"`
	EnableMapping     *bool  `json:"enable_mapping"`
	UseCloudSlam      *bool  `json:"use_cloud_slam"`
	// NewConfigFlag is used to access the config changes from imu and cloud story work while providing backwards compatibility
	// To be removed during JIRA Ticket: RSDK-4742 https://viam.atlassian.net/browse/RSDK-4742
	NewConfigFlag bool `json:"new_config_flag"`
}

// OptionalConfigParams holds the optional config parameters of SLAM.
type OptionalConfigParams struct {
	LidarDataRateMsec int
	ImuName           string
	ImuDataRateMsec   int
	MapRateSec        int
	EnableMapping     bool
	ExistingMap       string
}

var (
	errCameraMustHaveName           = errors.New("\"camera[name]\" is required")
	errSensorsMustNotBeEmpty        = errors.New("\"sensors\" must not be empty")
	errLocalizationInOfflineMode    = newError("data_rate_msec = 0 and enable_mapping = false. localization in offline mode not supported.")
	errLocalizationInOfflineModeIMU = newError("camera[data_freq_hz] and enable_mapping = false. localization in offline mode not supported.")
)

// Validate creates the list of implicit dependencies.
func (config *Config) Validate(path string) ([]string, error) {
	cameraName := ""
	imuName := ""
	imuExists := false
	if config.NewConfigFlag {
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

		imuName, imuExists = config.MovementSensor["name"]
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

	if !config.CloudStoryEnabled {
		if config.DataDirectory == "" {
			return nil, utils.NewConfigValidationFieldRequiredError(path, "data_dir")
		}

		if config.MapRateSec != nil && *config.MapRateSec < 0 {
			return nil, errors.New("cannot specify map_rate_sec less than zero")
		}
	}

	deps := []string{cameraName}
	if imuExists {
		deps = []string{cameraName, imuName}
	}
	return deps, nil
}

// GetOptionalParameters sets any unset optional config parameters to the values passed to this function,
// and returns them.
func GetOptionalParameters(config *Config, defaultLidarDataRateMsec, defaultIMUDataRateMsec, defaultMapRateSec int, logger golog.Logger,
) (OptionalConfigParams, error) {
	var optionalConfigParams OptionalConfigParams
	// feature flag for new config
	if config.NewConfigFlag {
		strCameraDataFreqHz, exists := config.Camera["data_frequency_hz"]
		if !exists {
			optionalConfigParams.LidarDataRateMsec = defaultLidarDataRateMsec
			logger.Debugf("config did not provide camera[data_frequency_hz], setting to default value of %d", 1000/defaultLidarDataRateMsec)
		} else {
			lidarDataFreqHz, err := strconv.Atoi(strCameraDataFreqHz)
			if err != nil {
				return OptionalConfigParams{}, newError("camera[data_frequency_hz] must only contain digits")
			}
			if lidarDataFreqHz != 0 {
				optionalConfigParams.LidarDataRateMsec = 1000 / lidarDataFreqHz
			}
		}
		imuName, exists := config.MovementSensor["name"]
		if exists {
			optionalConfigParams.ImuName = imuName
			strMovementSensorDataFreqHz, ok := config.MovementSensor["data_frequency_hz"]
			if !ok {
				if optionalConfigParams.LidarDataRateMsec == 0 {
					optionalConfigParams.ImuDataRateMsec = 0
					logger.Warn("camera[data_frequency_hz] is set to 0, " +
						"setting movement_sensor[data_frequency_hz] to 0")
				} else {
					optionalConfigParams.ImuDataRateMsec = defaultIMUDataRateMsec
					logger.Warnf("config did not provide movement_sensor[data_frequency_hz], "+
						"setting to default value of %d", 1000/defaultIMUDataRateMsec)
				}
			} else {
				imuDataFreqHz, err := strconv.Atoi(strMovementSensorDataFreqHz)
				if err != nil {
					return OptionalConfigParams{}, newError("movement_sensor[data_frequency_hz] must only contain digits")
				}
				if imuDataFreqHz != 0 {
					optionalConfigParams.ImuDataRateMsec = 1000 / imuDataFreqHz
				}
			}
		}
	} else {
		if config.DataRateMsec == nil {
			optionalConfigParams.LidarDataRateMsec = defaultLidarDataRateMsec
			logger.Debugf("no data_rate_msec given, setting to default value of %d", defaultLidarDataRateMsec)
		} else {
			optionalConfigParams.LidarDataRateMsec = *config.DataRateMsec
		}
	}

	if config.CloudStoryEnabled {
		if config.ExistingMap == "" {
			logger.Debug("no existing_map provided, entering mapping mode")
		} else {
			if !strings.HasSuffix(config.ExistingMap, ".pbstream") {
				return OptionalConfigParams{}, newError("existing map is not a .pbstream file")
			}
			optionalConfigParams.ExistingMap = config.ExistingMap
		}

		if config.EnableMapping == nil {
			logger.Debug("no enable_mapping given, setting to default value of false")
		} else {
			optionalConfigParams.EnableMapping = *config.EnableMapping
		}

		if err := validateModes(optionalConfigParams, config.NewConfigFlag); err != nil {
			return OptionalConfigParams{}, err
		}

		return optionalConfigParams, nil
	}

	if config.EnableMapping != nil && *config.EnableMapping {
		logger.Warn("enable_mapping set to true while cloud_story_enabled = false will not change any behavior")
	}

	if config.MapRateSec == nil {
		logger.Debugf("no map_rate_sec given, setting to default value of %d", defaultMapRateSec)
		optionalConfigParams.MapRateSec = defaultMapRateSec
	} else {
		optionalConfigParams.MapRateSec = *config.MapRateSec
	}

	return optionalConfigParams, nil
}

func validateModes(
	optionalConfigParams OptionalConfigParams,
	imuEnabled bool,
) error {
	offlineMode := optionalConfigParams.LidarDataRateMsec == 0
	localizationMode := !optionalConfigParams.EnableMapping
	if localizationMode && offlineMode {
		if imuEnabled {
			return errLocalizationInOfflineModeIMU
		}
		return errLocalizationInOfflineMode
	}
	return nil
}
