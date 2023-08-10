package config

import (
	"fmt"
	"testing"

	"github.com/edaniels/golog"
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

	t.Run(fmt.Sprintf("Empty config%s", suffix), func(t *testing.T) {
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

	t.Run(fmt.Sprintf("Simplest valid config%s", suffix), func(t *testing.T) {
		cfgService := makeCfgService(imuIntegrationEnabled, cloudStoryEnabled)
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run(fmt.Sprintf("Config without required fields%s", suffix), func(t *testing.T) {
		var requiredFields []string
		if imuIntegrationEnabled {
			requiredFields = []string{"data_dir", "camera"}
		} else {
			requiredFields = []string{"data_dir", "sensors"}
		}

		dataDirErr := utils.NewConfigValidationFieldRequiredError(testCfgPath, requiredFields[0])
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

	t.Run(fmt.Sprintf("Config with invalid parameter type%s", suffix), func(t *testing.T) {
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

	t.Run(fmt.Sprintf("Config with out of range values%s", suffix), func(t *testing.T) {
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
				"map_rate_sec":      "-1",
			}

			_, err = newConfig(cfgService)
			test.That(t, err, test.ShouldBeError, newError("cannot specify map_rate_sec less than zero"))
		} else {
			cfgService.Attributes["data_rate_msec"] = -1
			_, err := newConfig(cfgService)
			test.That(t, err, test.ShouldBeError, newError("cannot specify data_rate_msec less than zero"))

			cfgService.Attributes["data_rate_msec"] = 1
			cfgService.Attributes["map_rate_sec"] = -1
			_, err = newConfig(cfgService)

			test.That(t, err, test.ShouldBeError, newError("cannot specify map_rate_sec less than zero"))
		}
	})

	t.Run(fmt.Sprintf("All parameters e2e%s", suffix), func(t *testing.T) {
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

func TestValidate(t *testing.T) {
	testValidateTesthelper(
		t,
		false,
		false,
		" with imuIntegrationEnabled = false & cloudStoryEnabled = false")
	testValidateTesthelper(
		t,
		false,
		true,
		"with imuIntegrationEnabled = false & cloudStoryEnabled = true")
	testValidateTesthelper(
		t,
		true,
		false,
		" with imuIntegrationEnabled = true & cloudStoryEnabled = false")
	testValidateTesthelper(
		t,
		true,
		true,
		"with imuIntegrationEnabled = true & cloudStoryEnabled = true")
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

func TestGetOptionalParameters(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("Pass default parameters", func(t *testing.T) {
		cfgService := makeCfgService(false, false)
		cfgService.Attributes["sensors"] = []string{"a"}
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		lidarDataRateMsec, _, _, mapRateSec, enableMapping, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, mapRateSec, test.ShouldEqual, 1002)
		test.That(t, lidarDataRateMsec, test.ShouldEqual, 1000)
		test.That(t, enableMapping, test.ShouldBeFalse)
	})

	t.Run("Return overrides", func(t *testing.T) {
		cfgService := makeCfgService(false, false)
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "a",
			"data_frequency_hz": "1",
		}
		cfg, err := newConfig(cfgService)
		two := 2
		cfg.MapRateSec = &two
		dataRate := 50
		cfg.DataRateMsec = &dataRate
		test.That(t, err, test.ShouldBeNil)
		lidarDataRateMsec, _, _, mapRateSec, enableMapping, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, mapRateSec, test.ShouldEqual, 2)
		test.That(t, lidarDataRateMsec, test.ShouldEqual, 50)
		test.That(t, enableMapping, test.ShouldBeFalse)
	})
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

func TestGetOptionalParametersFeatureFlag(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("Pass default parameters with feature flag enabled", func(t *testing.T) {
		cfgService := makeCfgService(true, false)
		cfgService.Attributes["camera"] = map[string]string{"name": "a"}
		cfgService.Attributes["movement_sensor"] = map[string]string{"name": "b"}
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		lidarDataRateMsec, imuName, imuDataRateMsec, mapRateSec, enableMapping, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, mapRateSec, test.ShouldEqual, 1002)
		test.That(t, lidarDataRateMsec, test.ShouldEqual, 1000)
		test.That(t, imuName, test.ShouldEqual, "b")
		test.That(t, imuDataRateMsec, test.ShouldEqual, 1000)
		test.That(t, enableMapping, test.ShouldBeFalse)
	})

	t.Run("Return overrides with feature flag enabled", func(t *testing.T) {
		cfgService := makeCfgService(true, false)
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "a",
			"data_frequency_hz": "5",
		}
		cfgService.Attributes["movement_sensor"] = map[string]string{
			"name":              "b",
			"data_frequency_hz": "20",
		}
		cfg, err := newConfig(cfgService)
		two := 2
		cfg.MapRateSec = &two
		test.That(t, err, test.ShouldBeNil)
		lidarDataRateMsec, imuName, imuDataRateMsec, mapRateSec, enableMapping, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, mapRateSec, test.ShouldEqual, 2)
		test.That(t, lidarDataRateMsec, test.ShouldEqual, 200)
		test.That(t, imuName, test.ShouldEqual, "b")
		test.That(t, imuDataRateMsec, test.ShouldEqual, 50)
		test.That(t, enableMapping, test.ShouldBeFalse)
	})

	t.Run("Unit test return error if lidar data frequency is invalid", func(t *testing.T) {
		cfgService := makeCfgService(true, false)
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "a",
			"data_frequency_hz": "b",
		}
		cfg, err := newConfigWithoutValidate(cfgService)
		test.That(t, err, test.ShouldBeNil)
		_, _, _, _, _, err = GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeError, newError("camera[data_frequency_hz] must only contain digits"))
	})

	t.Run("Unit test return error if imu data frequency is invalid", func(t *testing.T) {
		cfgService := makeCfgService(true, false)
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
		_, _, _, _, _, err = GetOptionalParameters(
			cfg,
			1000,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeError, newError("movement_sensor[data_frequency_hz] must only contain digits"))
	})
}

func newConfigWithoutValidate(conf resource.Config) (*Config, error) {
	slamConf, err := resource.TransformAttributeMap[*Config](conf.Attributes)
	if err != nil {
		return &Config{}, newError(err.Error())
	}

	return slamConf, nil
}
