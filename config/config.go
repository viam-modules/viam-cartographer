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

	ExistingMap   string `json:"existing_map"`
	EnableMapping *bool  `json:"enable_mapping"`
	UseCloudSlam  *bool  `json:"use_cloud_slam"`
}

// OptionalConfigParams holds the optional config parameters of SLAM.
type OptionalConfigParams struct {
	LidarDataFrequencyHz int
	ImuName              string
	ImuDataFrequencyHz   int
	MapRateSec           int
	EnableMapping        bool
	ExistingMap          string
}

var (
	errCameraMustHaveName        = errors.New("\"camera[name]\" is required")
	errLocalizationInOfflineMode = newError("camera[data_freq_hz] and enable_mapping = false. localization in offline mode not supported.")
)

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

	imuName, imuExists := config.MovementSensor["name"]

	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	deps := []string{cameraName}
	if imuExists {
		deps = []string{cameraName, imuName}
	}
	return deps, nil
}

// GetOptionalParameters sets any unset optional config parameters to the values passed to this function,
// and returns them.
func GetOptionalParameters(config *Config, defaultLidarDataFrequencyHz, defaultIMUDataFrequencyHz int, logger golog.Logger,
) (OptionalConfigParams, error) {
	var optionalConfigParams OptionalConfigParams

	strCameraDataFreqHz, exists := config.Camera["data_frequency_hz"]
	if !exists {
		optionalConfigParams.LidarDataFrequencyHz = defaultLidarDataFrequencyHz
		logger.Debugf("config did not provide camera[data_frequency_hz], setting to default value of %d", defaultLidarDataFrequencyHz)
	} else {
		lidarDataFreqHz, err := strconv.Atoi(strCameraDataFreqHz)
		if err != nil {
			return OptionalConfigParams{}, newError("camera[data_frequency_hz] must only contain digits")
		}
		if lidarDataFreqHz != 0 {
			optionalConfigParams.LidarDataFrequencyHz = lidarDataFreqHz
		}
	}
	imuName, exists := config.MovementSensor["name"]
	if exists {
		optionalConfigParams.ImuName = imuName
		strMovementSensorDataFreqHz, ok := config.MovementSensor["data_frequency_hz"]
		if !ok {
			if optionalConfigParams.LidarDataFrequencyHz == 0 {
				optionalConfigParams.ImuDataFrequencyHz = 0
				logger.Warn("camera[data_frequency_hz] is set to 0, " +
					"setting movement_sensor[data_frequency_hz] to 0")
			} else {
				optionalConfigParams.ImuDataFrequencyHz = defaultIMUDataFrequencyHz
				logger.Warnf("config did not provide movement_sensor[data_frequency_hz], "+
					"setting to default value of %d", defaultIMUDataFrequencyHz)
			}
		} else {
			imuDataFreqHz, err := strconv.Atoi(strMovementSensorDataFreqHz)
			if err != nil {
				return OptionalConfigParams{}, newError("movement_sensor[data_frequency_hz] must only contain digits")
			}
			if imuDataFreqHz != 0 {
				optionalConfigParams.ImuDataFrequencyHz = imuDataFreqHz
			}
		}
	}
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

	if err := validateModes(optionalConfigParams); err != nil {
		return OptionalConfigParams{}, err
	}

	if config.EnableMapping != nil && *config.EnableMapping {
		logger.Warn("enable_mapping set to true while cloud_story_enabled = false will not change any behavior")
	}

	return optionalConfigParams, nil
}

func validateModes(optionalConfigParams OptionalConfigParams) error {
	offlineMode := optionalConfigParams.LidarDataFrequencyHz == 0
	localizationMode := !optionalConfigParams.EnableMapping
	if localizationMode && offlineMode {
		return errLocalizationInOfflineMode
	}
	return nil
}
