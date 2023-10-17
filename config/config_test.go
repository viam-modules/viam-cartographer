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

func TestValidate(t *testing.T) {
	testCfgPath := "services.slam.attributes.fake"
	logger := golog.NewTestLogger(t)

	t.Run("Empty config", func(t *testing.T) {
		model := resource.DefaultModelFamily.WithModel("test")
		cfgService := resource.Config{Name: "test", API: slam.API, Model: model}
		cfgService.Attributes = make(map[string]interface{})

		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeError, newError("error validating \"services.slam.attributes.fake\": \"camera[name]\" is required"))
	})

	t.Run("Simplest valid config", func(t *testing.T) {
		cfgService := makeCfgService()
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Config without required fields", func(t *testing.T) {
		requiredFields := []string{"camera"}
		expectedErrors := map[string]error{
			"camera": newError(utils.NewConfigValidationError(testCfgPath, errCameraMustHaveName).Error()),
		}

		for _, requiredField := range requiredFields {
			logger.Debugf("Testing SLAM config without %s\n", requiredField)
			cfgService := makeCfgService()
			delete(cfgService.Attributes, requiredField)
			_, err := newConfig(cfgService)

			test.That(t, err, test.ShouldBeError, expectedErrors[requiredField])
		}

		// Test for missing config_params attributes
		logger.Debug("Testing SLAM config without config_params[mode]")
		cfgService := makeCfgService()
		delete(cfgService.Attributes["config_params"].(map[string]string), "mode")
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeError, newError(utils.NewConfigValidationFieldRequiredError(testCfgPath, "config_params[mode]").Error()))
	})

	t.Run("Config with invalid parameter type", func(t *testing.T) {
		key := "existing_map"

		cfgService := makeCfgService()
		cfgService.Attributes[key] = true

		_, err := newConfig(cfgService)
		msg := fmt.Sprintf("1 error(s) decoding:\n\n* '%s' expected type 'string', got unconvertible type 'bool', value: 'true'", key)
		expE := newError(msg)
		test.That(t, err, test.ShouldBeError, expE)

		cfgService.Attributes[key] = "true"
		_, err = newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Config with out of range values", func(t *testing.T) {
		cfgService := makeCfgService()
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "a",
			"data_frequency_hz": "-1",
		}
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeError, newError("cannot specify camera[data_frequency_hz] less than zero"))
	})

	t.Run("All parameters e2e", func(t *testing.T) {
		cfgService := makeCfgService()
		cfgService.Attributes["camera"] = map[string]string{"name": "test", "data_frequency_hz": "10"}

		cfgService.Attributes["config_params"] = map[string]string{
			"mode":    "test mode",
			"value":   "0",
			"value_2": "test",
		}
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, cfg.ConfigParams, test.ShouldResemble, cfgService.Attributes["config_params"])
	})
}

// makeCfgService creates the simplest possible config that can pass validation.
func makeCfgService() resource.Config {
	model := resource.DefaultModelFamily.WithModel("test")
	cfgService := resource.Config{Name: "test", API: slam.API, Model: model}
	cfgService.Attributes = map[string]interface{}{
		"config_params": map[string]string{"mode": "test mode"},
	}

	cfgService.Attributes["camera"] = map[string]string{
		"name": "a",
	}

	return cfgService
}

func TestGetOptionalParameters(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("Pass default parameters", func(t *testing.T) {
		cfgService := makeCfgService()
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		optionalConfigParams, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, optionalConfigParams.LidarDataFrequencyHz, test.ShouldEqual, 1000)
		test.That(t, optionalConfigParams.ImuName, test.ShouldEqual, "")
		test.That(t, optionalConfigParams.ImuDataFrequencyHz, test.ShouldEqual, 0)
		test.That(t, optionalConfigParams.EnableMapping, test.ShouldBeFalse)
		test.That(t, optionalConfigParams.MapRateSec, test.ShouldEqual, 0)
		test.That(t, optionalConfigParams.ExistingMap, test.ShouldEqual, "")
	})

	t.Run("Pass default parameters with no IMU name specified", func(t *testing.T) {
		cfgService := makeCfgService()

		cfgService.Attributes["movement_sensor"] = map[string]string{
			"name":              "",
			"data_frequency_hz": "5",
		}

		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		optionalConfigParams, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, optionalConfigParams.LidarDataFrequencyHz, test.ShouldEqual, 1000)
		test.That(t, optionalConfigParams.ImuName, test.ShouldEqual, "")
		test.That(t, optionalConfigParams.ImuDataFrequencyHz, test.ShouldEqual, 0)
		test.That(t, optionalConfigParams.EnableMapping, test.ShouldBeFalse)
		test.That(t, optionalConfigParams.MapRateSec, test.ShouldEqual, 0)
		test.That(t, optionalConfigParams.ExistingMap, test.ShouldEqual, "")
	})

	t.Run("Return overrides", func(t *testing.T) {
		cfgService := makeCfgService()

		cfgService.Attributes["camera"] = map[string]string{
			"name":              "testNameCamera",
			"data_frequency_hz": "2",
		}
		cfgService.Attributes["movement_sensor"] = map[string]string{
			"name":              "testNameSensor",
			"data_frequency_hz": "2",
		}

		cfgService.Attributes["enable_mapping"] = true

		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		optionalConfigParams, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, optionalConfigParams.ImuName, test.ShouldEqual, "testNameSensor")
		test.That(t, optionalConfigParams.ImuDataFrequencyHz, test.ShouldEqual, 2)
		test.That(t, optionalConfigParams.LidarDataFrequencyHz, test.ShouldEqual, 2)
		test.That(t, optionalConfigParams.EnableMapping, test.ShouldBeTrue)
	})

	t.Run("Pass invalid existing map", func(t *testing.T) {
		cfgService := makeCfgService()
		cfgService.Attributes["existing_map"] = "test-file"
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		optionalConfigParams, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			logger)
		test.That(t, err, test.ShouldBeError, newError("existing map is not a .pbstream file"))
		test.That(t, optionalConfigParams, test.ShouldResemble, OptionalConfigParams{})
	})

	t.Run("config that puts cartographer in offline mode and in localization mode", func(t *testing.T) {
		cfgService := makeCfgService()
		cfgService.Attributes["existing_map"] = "test-file.pbstream"
		cfgService.Attributes["enable_mapping"] = false
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "testcam",
			"data_frequency_hz": "0",
		}

		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		optionalConfigParams, err := GetOptionalParameters(
			cfg,
			1000,
			1000,
			logger)
		test.That(t, err, test.ShouldBeError, errLocalizationInOfflineMode)
		test.That(t, optionalConfigParams, test.ShouldResemble, OptionalConfigParams{})
	})

	sensorAttributeTestHelper(t, logger)
}

func sensorAttributeTestHelper(
	t *testing.T,
	logger *zap.SugaredLogger,
) {
	t.Run("Unit test return error if lidar data frequency is invalid", func(t *testing.T) {
		cfgService := makeCfgService()
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
			logger)
		test.That(t, err, test.ShouldBeError, newError("camera[data_frequency_hz] must only contain digits"))
	})

	t.Run("Unit test return error if imu data frequency is invalid", func(t *testing.T) {
		cfgService := makeCfgService()
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
			logger)
		test.That(t, err, test.ShouldBeError, newError("movement_sensor[data_frequency_hz] must only contain digits"))
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

func newConfigWithoutValidate(conf resource.Config) (*Config, error) {
	slamConf, err := resource.TransformAttributeMap[*Config](conf.Attributes)
	if err != nil {
		return &Config{}, newError(err.Error())
	}

	return slamConf, nil
}
