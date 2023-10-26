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
	LidarDataFrequencyHz          int
	MovementSensorName            string
	MovementSensorDataFrequencyHz int
	MapRateSec                    int
	EnableMapping                 bool
	ExistingMap                   string
}

var (
	errCameraMustHaveName        = errors.New("\"camera[name]\" is required")
	errLocalizationInOfflineMode = newError("camera[data_freq_hz] and enable_mapping = false. localization in offline mode not supported.")
)

// Validate creates the list of implicit dependencies.
func (config *Config) Validate(path string) ([]string, error) {
	var deps []string
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
	deps = append(deps, cameraName)

	movementSensorName, movementSensorExists := config.MovementSensor["name"]
	if movementSensorExists && movementSensorName != "" {
		deps = append(deps, movementSensorName)
	}

	if config.ConfigParams["mode"] == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "config_params[mode]")
	}

	return deps, nil
}

// GetOptionalParameters sets any unset optional config parameters to the values passed to this function,
// and returns them.
func GetOptionalParameters(config *Config, defaultLidarDataFrequencyHz, defaultMovementSensorDataFrequencyHz int, logger golog.Logger,
) (OptionalConfigParams, error) {
	var optionalConfigParams OptionalConfigParams

	// Validate camera info and set defaults
	if strCameraDataFreqHz, exists := config.Camera["data_frequency_hz"]; !exists {
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

	// Validate movement sensor info and set defaults
	if movementSensorName, exists := config.MovementSensor["name"]; exists && movementSensorName != "" {
		optionalConfigParams.MovementSensorName = movementSensorName
		if strMovementSensorDataFreqHz, ok := config.MovementSensor["data_frequency_hz"]; !ok {
			if optionalConfigParams.LidarDataFrequencyHz == 0 {
				optionalConfigParams.MovementSensorDataFrequencyHz = 0
				logger.Warn("camera[data_frequency_hz] is set to 0, " +
					"setting movement_sensor[data_frequency_hz] to 0")
			} else {
				optionalConfigParams.MovementSensorDataFrequencyHz = defaultMovementSensorDataFrequencyHz
				logger.Warnf("config did not provide movement_sensor[data_frequency_hz], "+
					"setting to default value of %d", defaultMovementSensorDataFrequencyHz)
			}
		} else {
			movementSensorDataFreqHz, err := strconv.Atoi(strMovementSensorDataFreqHz)
			if err != nil {
				return OptionalConfigParams{}, newError("movement_sensor[data_frequency_hz] must only contain digits")
			}
			if movementSensorDataFreqHz != 0 {
				optionalConfigParams.MovementSensorDataFrequencyHz = movementSensorDataFreqHz
			}
		}
	}

	// Check if apriori map exists and is in correct format
	if config.ExistingMap == "" {
		logger.Debug("no existing_map provided, entering mapping mode")
	} else {
		if !strings.HasSuffix(config.ExistingMap, ".pbstream") {
			return OptionalConfigParams{}, newError("existing map is not a .pbstream file")
		}
		optionalConfigParams.ExistingMap = config.ExistingMap
	}

	// Setting enable mapping
	if config.EnableMapping == nil {
		logger.Debug("no enable_mapping given, setting to default value of false")
	} else {
		optionalConfigParams.EnableMapping = *config.EnableMapping
	}

	// Validate slam mode
	if err := validateModes(optionalConfigParams); err != nil {
		return OptionalConfigParams{}, err
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
