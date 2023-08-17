package config

import (
	"fmt"
	"testing"

	"github.com/edaniels/golog"
	"go.uber.org/zap"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/test"
	"go.viam.com/utils"
)

func testValidateTesthelper(
	t *testing.T,
	imuIntegrationEnabled bool,
	cloudStoryEnabled bool,
	suffix string,
) {
	testCfgPath := "services.slam.attributes.fake"
	logger := golog.NewTestLogger(t)

	t.Run(fmt.Sprintf("Empty config %s", suffix), func(t *testing.T) {
		model := resource.DefaultModelFamily.WithModel("test")
		cfgService := resource.Config{Name: "test", API: slam.API, Model: model}
		if imuIntegrationEnabled || cloudStoryEnabled {
			cfgService.Attributes = make(map[string]interface{})
		}

		if imuIntegrationEnabled {
			cfgService.Attributes["imu_integration_enabled"] = true
		}

		if cloudStoryEnabled {
			cfgService.Attributes["cloud_story_enabled"] = true
		}
		_, err := newConfig(cfgService)

		if imuIntegrationEnabled {
			test.That(t, err, test.ShouldBeError, newError("error validating \"services.slam.attributes.fake\": \"camera[name]\" is required"))
		} else {
			test.That(t, err, test.ShouldBeError, newError("error validating \"services.slam.attributes.fake\": \"sensors\" must not be empty"))
		}
	})

	t.Run(fmt.Sprintf("Simplest valid config %s", suffix), func(t *testing.T) {
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run(fmt.Sprintf("Config without required fields %s", suffix), func(t *testing.T) {
		requiredFields := getRequiredFields(imuIntegrationEnabled, cloudStoryEnabled)

		dataDirErr := utils.NewConfigValidationFieldRequiredError(testCfgPath, "data_dir")
		sensorErr := utils.NewConfigValidationError(testCfgPath, errSensorsMustNotBeEmpty)
		cameraErr := utils.NewConfigValidationError(testCfgPath, errCameraMustHaveName)

		expectedErrors := map[string]error{
			"data_dir": newError(dataDirErr.Error()),
			"sensors":  newError(sensorErr.Error()),
			"camera":   newError(cameraErr.Error()),
		}
		for _, requiredField := range requiredFields {
			logger.Debugf("Testing SLAM config without %s\n", requiredField)
			cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
			delete(cfgService.Attributes, requiredField)
			_, err := newConfig(cfgService)

			test.That(t, err, test.ShouldBeError, expectedErrors[requiredField])
		}
		// Test for missing config_params attributes
		logger.Debug("Testing SLAM config without config_params[mode]")
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		delete(cfgService.Attributes["config_params"].(map[string]string), "mode")
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeError, newError(utils.NewConfigValidationFieldRequiredError(testCfgPath, "config_params[mode]").Error()))
	})

	t.Run(fmt.Sprintf("Config with invalid parameter type %s", suffix), func(t *testing.T) {
		key := "data_dir"

		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		cfgService.Attributes[key] = true

		_, err := newConfig(cfgService)
		msg := fmt.Sprintf("1 error(s) decoding:\n\n* '%s' expected type 'string', got unconvertible type 'bool', value: 'true'", key)
		expE := newError(msg)
		test.That(t, err, test.ShouldBeError, expE)

		cfgService.Attributes[key] = "true"
		_, err = newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run(fmt.Sprintf("Config with out of range values %s", suffix), func(t *testing.T) {
		var mapRateSecError error
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		if imuIntegrationEnabled {
			cfgService.Attributes["camera"] = map[string]string{
				"name":              "a",
				"data_frequency_hz": "-1",
			}
			_, err := newConfig(cfgService)
			test.That(t, err, test.ShouldBeError, newError("cannot specify camera[data_frequency_hz] less than zero"))

			cfgService.Attributes["camera"] = map[string]string{
				"name":              "a",
				"data_frequency_hz": "1",
			}
			cfgService.Attributes["map_rate_sec"] = -1

			_, mapRateSecError = newConfig(cfgService)
		} else {
			cfgService.Attributes["data_rate_msec"] = -1
			_, err := newConfig(cfgService)
			test.That(t, err, test.ShouldBeError, newError("cannot specify data_rate_msec less than zero"))

			cfgService.Attributes["data_rate_msec"] = 1
			cfgService.Attributes["map_rate_sec"] = -1
			_, mapRateSecError = newConfig(cfgService)
		}

		if cloudStoryEnabled {
			test.That(t, mapRateSecError, test.ShouldBeNil)
		} else {
			test.That(t, mapRateSecError, test.ShouldBeError, newError("cannot specify map_rate_sec less than zero"))
		}
	})

	t.Run(fmt.Sprintf("All parameters e2e %s", suffix), func(t *testing.T) {
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		cfgService.Attributes["sensors"] = []string{"a", "b"}
		cfgService.Attributes["data_rate_msec"] = 1001
		cfgService.Attributes["map_rate_sec"] = 1002

		cfgService.Attributes["config_params"] = map[string]string{
			"mode":    "test mode",
			"value":   "0",
			"value_2": "test",
		}
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, cfg.Sensors, test.ShouldResemble, cfgService.Attributes["sensors"])
		test.That(t, *cfg.DataRateMsec, test.ShouldEqual, cfgService.Attributes["data_rate_msec"])
		test.That(t, cfg.ConfigParams, test.ShouldResemble, cfgService.Attributes["config_params"])
		test.That(t, cfg.DataDirectory, test.ShouldEqual, cfgService.Attributes["data_dir"])
		test.That(t, *cfg.MapRateSec, test.ShouldEqual, cfgService.Attributes["map_rate_sec"])
	})
}

func getRequiredFields(imuIntegrationEnabled, cloudStoryEnabled bool) []string {
	requiredFields := []string{}

	if imuIntegrationEnabled {
		requiredFields = append(requiredFields, "camera")
	} else {
		requiredFields = append(requiredFields, "sensors")
	}

	if !cloudStoryEnabled {
		requiredFields = append(requiredFields, "data_dir")
	}
	return requiredFields
}

func TestValidate(t *testing.T) {
	for _, imuEnabled := range []bool{true, false} {
		for _, cloudStoryEnabled := range []bool{true, false} {
			suffix := fmt.Sprintf("with imuIntegrationEnabled = %t & cloudStoryEnabled = %t", imuEnabled, cloudStoryEnabled)
			testValidateTesthelper(t, imuEnabled, cloudStoryEnabled, suffix)
		}
	}
}

// makeCfgService creates the simplest possible config that can pass validation.
func makeCfgService(IMUIntegrationEnabled, cloudStoryEnabled bool) resource.Config {
	model := resource.DefaultModelFamily.WithModel("test")
	cfgService := resource.Config{Name: "test", API: slam.API, Model: model}
	cfgService.Attributes = map[string]interface{}{
		"config_params": map[string]string{"mode": "test mode"},
		"data_dir":      "path",
	}

	if IMUIntegrationEnabled {
		cfgService.Attributes["imu_integration_enabled"] = true
		cfgService.Attributes["camera"] = map[string]string{
			"name": "a",
		}
	} else {
		cfgService.Attributes["sensors"] = []string{"a"}
	}

	if cloudStoryEnabled {
		cfgService.Attributes["cloud_story_enabled"] = true
	}

	return cfgService
}

func getOptionalParametersTestHelper(
	t *testing.T,
	imuIntegrationEnabled bool,
	cloudStoryEnabled bool,
	suffix string,
) {
	logger := golog.NewTestLogger(t)

	t.Run(fmt.Sprintf("Pass default parameters %s", suffix), func(t *testing.T) {
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		cfgService.Attributes["sensors"] = []string{"a"}
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		optionalConfigParams, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, optionalConfigParams.LidarDataRateMsec, test.ShouldEqual, 1000)
		test.That(t, optionalConfigParams.EnableMapping, test.ShouldBeFalse)
		if cloudStoryEnabled {
			test.That(t, optionalConfigParams.MapRateSec, test.ShouldEqual, 0)
			test.That(t, optionalConfigParams.ExistingMap, test.ShouldEqual, "")
		} else {
			test.That(t, optionalConfigParams.MapRateSec, test.ShouldEqual, 1002)
		}
	})

	t.Run(fmt.Sprintf("Return overrides %s", suffix), func(t *testing.T) {
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		if imuIntegrationEnabled {
			cfgService.Attributes["camera"] = map[string]string{
				"name":              "testNameCamera",
				"data_frequency_hz": "2",
			}
			cfgService.Attributes["movement_sensor"] = map[string]string{
				"name":              "testNameSensor",
				"data_frequency_hz": "2",
			}
		}

		if cloudStoryEnabled {
			cfgService.Attributes["enable_mapping"] = true
		}

		cfg, err := newConfig(cfgService)
		two := 2
		cfg.MapRateSec = &two
		dataRate := 50
		cfg.DataRateMsec = &dataRate
		test.That(t, err, test.ShouldBeNil)
		optionalConfigParams, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeNil)
		if imuIntegrationEnabled {
			test.That(t, optionalConfigParams.ImuName, test.ShouldEqual, "testNameSensor")
			test.That(t, optionalConfigParams.ImuDataRateMsec, test.ShouldEqual, 500)
			test.That(t, optionalConfigParams.LidarDataRateMsec, test.ShouldEqual, 500)
		} else {
			test.That(t, optionalConfigParams.LidarDataRateMsec, test.ShouldEqual, 50)
		}

		if cloudStoryEnabled {
			test.That(t, optionalConfigParams.MapRateSec, test.ShouldEqual, 0)
			test.That(t, optionalConfigParams.EnableMapping, test.ShouldBeTrue)
		} else {
			test.That(t, optionalConfigParams.MapRateSec, test.ShouldEqual, 2)
			test.That(t, optionalConfigParams.EnableMapping, test.ShouldBeFalse)
		}
	})

	t.Run(fmt.Sprintf("Pass invalid existing map %s", suffix), func(t *testing.T) {
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		cfgService.Attributes["existing_map"] = "test-file"
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		optionalConfigParams, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		if cloudStoryEnabled {
			test.That(t, err, test.ShouldBeError, newError("existing map is not a .pbstream file"))
		} else {
			test.That(t, err, test.ShouldBeNil)
			test.That(t, optionalConfigParams.LidarDataRateMsec, test.ShouldEqual, 1000)
			test.That(t, optionalConfigParams.EnableMapping, test.ShouldBeFalse)
			test.That(t, optionalConfigParams.MapRateSec, test.ShouldEqual, 1002)
		}
	})

	t.Run(fmt.Sprintf("config that puts cartographer in offline mode and in localization mode %s", suffix), func(t *testing.T) {
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		cfgService.Attributes["existing_map"] = "test-file.pbstream"
		cfgService.Attributes["enable_mapping"] = false
		if imuIntegrationEnabled {
			cfgService.Attributes["camera"] = map[string]string{
				"name":              "testcam",
				"data_frequency_hz": "0",
			}
		} else {
			cfgService.Attributes["data_rate_msec"] = 0
		}
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		optionalConfigParams, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		if cloudStoryEnabled {
			if imuIntegrationEnabled {
				test.That(t, err, test.ShouldBeError, errLocalizationInOfflineModeIMU)
			} else {
				test.That(t, err, test.ShouldBeError, errLocalizationInOfflineMode)
			}
		} else {
			test.That(t, err, test.ShouldBeNil)
			test.That(t, optionalConfigParams.LidarDataRateMsec, test.ShouldEqual, 0)
			test.That(t, optionalConfigParams.EnableMapping, test.ShouldBeFalse)
			test.That(t, optionalConfigParams.MapRateSec, test.ShouldEqual, 1002)
		}
	})

	if imuIntegrationEnabled {
		sensorAttributeTestHelper(t, logger, imuIntegrationEnabled, cloudStoryEnabled)
	}
}

func sensorAttributeTestHelper(
	t *testing.T,
	logger *zap.SugaredLogger,
	imuIntegrationEnabled bool,
	cloudStoryEnabled bool,
) {
	t.Run("Unit test return error if lidar data frequency is invalid", func(t *testing.T) {
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "a",
			"data_frequency_hz": "b",
		}
		cfg, err := newConfigWithoutValidate(cfgService)
		test.That(t, err, test.ShouldBeNil)
		_, err = GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeError, newError("camera[data_frequency_hz] must only contain digits"))
	})

	t.Run("Unit test return error if imu data frequency is invalid", func(t *testing.T) {
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "a",
			"data_frequency_hz": "1",
		}
		cfgService.Attributes["movement_sensor"] = map[string]string{
			"name":              "b",
			"data_frequency_hz": "c",
		}
		cfg, err := newConfigWithoutValidate(cfgService)
		test.That(t, err, test.ShouldBeNil)
		_, err = GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeError, newError("movement_sensor[data_frequency_hz] must only contain digits"))
	})
}

func TestGetOptionalParameters(t *testing.T) {
	for _, imuEnabled := range []bool{true, false} {
		for _, cloudStoryEnabled := range []bool{true, false} {
			suffix := fmt.Sprintf("with imuIntegrationEnabled = %t & cloudStoryEnabled = %t", imuEnabled, cloudStoryEnabled)
			getOptionalParametersTestHelper(t, imuEnabled, cloudStoryEnabled, suffix)
		}
	}
}

func newConfig(conf resource.Config) (*Config, error) {
	slamConf, err := resource.TransformAttributeMap[*Config](conf.Attributes)
	if err != nil {
		return &Config{}, newError(err.Error())
	}

	if _, err := slamConf.Validate("services.slam.attributes.fake"); err != nil {
		return &Config{}, newError(err.Error())
	}

	return slamConf, nil
}

func newConfigWithoutValidate(conf resource.Config) (*Config, error) {
	slamConf, err := resource.TransformAttributeMap[*Config](conf.Attributes)
	if err != nil {
		return &Config{}, newError(err.Error())
	}

	return slamConf, nil
}
