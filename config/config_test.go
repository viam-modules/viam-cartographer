package config

import (
	"testing"

	"github.com/edaniels/golog"
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
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeError, newError("error validating \"services.slam.attributes.fake\": \"camera[name]\" is required"))
	})

	t.Run("Simplest valid config", func(t *testing.T) {
		cfgService := makeCfgService()
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Config without required fields", func(t *testing.T) {
		requiredFields := []string{"data_dir", "camera"}
		dataDirErr := utils.NewConfigValidationFieldRequiredError(testCfgPath, requiredFields[0])
		cameraErr := utils.NewConfigValidationError(testCfgPath, errCameraMustHaveName)

		expectedErrors := []error{
			newError(dataDirErr.Error()),
			newError(cameraErr.Error()),
		}
		for i, requiredField := range requiredFields {
			logger.Debugf("Testing SLAM config without %s\n", requiredField)
			cfgService := makeCfgService()
			delete(cfgService.Attributes, requiredField)
			_, err := newConfig(cfgService)

			test.That(t, err, test.ShouldBeError, expectedErrors[i])
		}
		// Test for missing config_params attributes
		logger.Debug("Testing SLAM config without config_params[mode]")
		cfgService := makeCfgService()
		delete(cfgService.Attributes["config_params"].(map[string]string), "mode")
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeError, newError(utils.NewConfigValidationFieldRequiredError(testCfgPath, "config_params[mode]").Error()))
	})

	t.Run("Config without camera name", func(t *testing.T) {
		// Test for missing camera name attribute
		logger.Debug("Testing SLAM config without camera[name]")
		cfgService := makeCfgService()
		delete(cfgService.Attributes["camera"].(map[string]string), "name")
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeError, newError(utils.NewConfigValidationFieldRequiredError(testCfgPath, "camera[name]").Error()))
	})

	t.Run("Config with invalid parameter type", func(t *testing.T) {
		cfgService := makeCfgService()
		cfgService.Attributes["data_dir"] = true
		_, err := newConfig(cfgService)
		expE := newError("1 error(s) decoding:\n\n* 'data_dir' expected type 'string', got unconvertible type 'bool', value: 'true'")
		test.That(t, err, test.ShouldBeError, expE)
		cfgService.Attributes["data_dir"] = "true"
		_, err = newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Config with invalid camera data_frequency_hz", func(t *testing.T) {
		cfgService := makeCfgService()
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "a",
			"data_frequency_hz": "twenty",
		}
		_, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeError, newError("camera[data_frequency_hz] must only contain digits"))
	})

	t.Run("Config with out of range values", func(t *testing.T) {
		cfgService := makeCfgService()
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
		_, err = newConfig(cfgService)
		test.That(t, err, test.ShouldBeError, newError("cannot specify map_rate_sec less than zero"))
	})

	t.Run("All parameters e2e", func(t *testing.T) {
		cfgService := makeCfgService()
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "a",
			"data_frequency_hz": "20",
		}
		cfgService.Attributes["map_rate_sec"] = 1002

		cfgService.Attributes["config_params"] = map[string]string{
			"mode":    "test mode",
			"value":   "0",
			"value_2": "test",
		}
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, cfg.DataDirectory, test.ShouldEqual, cfgService.Attributes["data_dir"])
		test.That(t, cfg.Camera, test.ShouldResemble, cfgService.Attributes["camera"])
		test.That(t, *cfg.MapRateSec, test.ShouldEqual, cfgService.Attributes["map_rate_sec"])
		test.That(t, cfg.ConfigParams, test.ShouldResemble, cfgService.Attributes["config_params"])
	})
}

// makeCfgService creates the simplest possible config that can pass validation.
func makeCfgService() resource.Config {
	model := resource.DefaultModelFamily.WithModel("test")
	cfgService := resource.Config{Name: "test", API: slam.API, Model: model}
	cfgService.Attributes = make(map[string]interface{})
	cfgService.Attributes["config_params"] = map[string]string{
		"mode": "test mode",
	}
	cfgService.Attributes["data_dir"] = "path"
	cfgService.Attributes["camera"] = map[string]string{
		"name":              "a",
		"data_frequency_hz": "20",
	}
	return cfgService
}

func TestGetOptionalParameters(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("Pass default parameters", func(t *testing.T) {
		cfgService := makeCfgService()
		cfgService.Attributes["camera"] = map[string]string{"name": "a"}
		cfg, err := newConfig(cfgService)
		test.That(t, err, test.ShouldBeNil)
		lidarDataRateMsec, mapRateSec, err := GetOptionalParameters(
			cfg,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, mapRateSec, test.ShouldEqual, 1002)
		test.That(t, lidarDataRateMsec, test.ShouldEqual, 1000)
	})

	t.Run("Return overrides", func(t *testing.T) {
		cfgService := makeCfgService()
		cfgService.Attributes["camera"] = map[string]string{
			"name":              "a",
			"data_frequency_hz": "1",
		}
		cfg, err := newConfig(cfgService)
		two := 2
		cfg.MapRateSec = &two
		cfg.Camera["data_frequency_hz"] = "20"
		test.That(t, err, test.ShouldBeNil)
		lidarDataRateMsec, mapRateSec, err := GetOptionalParameters(
			cfg,
			1000,
			1002,
			logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, mapRateSec, test.ShouldEqual, 2)
		test.That(t, lidarDataRateMsec, test.ShouldEqual, 50)
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
