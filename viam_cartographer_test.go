// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user. It also runs integration tests
// that test the interaction with the core C++ viam-cartographer code and the Golang implementation of the
// cartographer slam service.
package viamcartographer_test

import (
	"context"
	"os"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	viamgrpc "go.viam.com/rdk/grpc"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/test"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	"github.com/viamrobotics/viam-cartographer/cartofacade"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	"github.com/viamrobotics/viam-cartographer/testhelper"
)

const (
	testExecutableName = "true" // the program "true", not the boolean value
	testDataFreqHz     = "5"
)

var (
	testMapRateSec = 200
	_zeroInt       = 0
	_zeroTime      = time.Time{}
)

func TestNew(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("Failed creation of cartographer slam service with more than one sensor", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)

		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()
		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "lidar", "name2": "one-too-many"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		_, err = testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: 'sensors' must contain only one "+
				"lidar camera, but is 'sensors: [lidar, one-too-many]'"))
	})

	t.Run("Failed creation of cartographer slam service with non-existing sensor", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "gibberish", "data_frequency_hz": testDataFreqHz},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		_, err = testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				"gibberish for slam service: \"rdk:component:camera/gibberish\" missing from dependencies"))
	})

	t.Run("Successful creation of cartographer slam service with good lidar", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "good_lidar", "data_frequency_hz": testDataFreqHz},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with invalid sensor "+
		"that errors during call to NextPointCloud", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "invalid_sensor", "data_frequency_hz": testDataFreqHz},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		_, err = testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,

			errors.New("failed to get data from lidar: ValidateGetData timeout: NextPointCloud error: invalid sensor"))
	})

	t.Run("Successful creation of cartographer slam service in localization mode", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			MapRateSec:    &_zeroInt,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		cs, ok := svc.(*viamcartographer.CartographerService)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, cs.SlamMode, test.ShouldEqual, cartofacade.LocalizingMode)

		timestamp1, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)
		_, err = svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)
		timestamp2, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp1, test.ShouldResemble, timestamp2)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service in non localization mode", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			MapRateSec:    &testMapRateSec,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		cs, ok := svc.(*viamcartographer.CartographerService)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, cs.SlamMode, test.ShouldEqual, cartofacade.UpdatingMode)

		timestamp1, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)
		_, err = svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)
		timestamp2, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp2.After(timestamp1), test.ShouldBeTrue)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Fails to create cartographer slam service with no sensor when feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError, errors.New("error validating \"path\": \"sensors\" must not be empty"))
		test.That(t, svc, test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with more than one sensor when feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "lidar", "name2": "one-too-many"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		_, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: 'sensors' must contain only one "+
				"lidar camera, but is 'sensors: [lidar, one-too-many]'"))
	})

	t.Run("Failed creation of cartographer slam service with non-existing sensor when feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "gibberish"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		_, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("error getting lidar camera "+
				"gibberish for slam service: \"rdk:component:camera/gibberish\" missing from dependencies"))
	})

	t.Run("Successful creation of cartographer slam service with good lidar when feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "good_lidar", "data_frequency_hz": testDataFreqHz},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with invalid sensor "+
		"that errors during call to NextPointCloud when feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "invalid_sensor", "data_frequency_hz": testDataFreqHz},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		_, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,
			errors.New("failed to get data from lidar: ValidateGetData timeout: NextPointCloud error: invalid sensor"))
	})

	t.Run("Successful creation of cartographer slam service in localization mode when feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "replay_sensor"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			MapRateSec:    &_zeroInt,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		_, componentReference, err := svc.GetPosition(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, componentReference, test.ShouldEqual, "replay_sensor")

		timestamp1, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcmFunc, err := svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcm, err := slam.HelperConcatenateChunksToFull(pcmFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcm, test.ShouldNotBeNil)

		isFunc, err := svc.GetInternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)

		is, err := slam.HelperConcatenateChunksToFull(isFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, is, test.ShouldNotBeNil)

		timestamp2, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp1, test.ShouldResemble, timestamp2)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service in non localization mode when feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "good_lidar", "data_frequency_hz": testDataFreqHz},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		_, componentReference, err := svc.GetPosition(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, componentReference, test.ShouldEqual, "good_lidar")

		timestamp1, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcmFunc, err := svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcm, err := slam.HelperConcatenateChunksToFull(pcmFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcm, test.ShouldNotBeNil)

		isFunc, err := svc.GetInternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)

		is, err := slam.HelperConcatenateChunksToFull(isFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, is, test.ShouldNotBeNil)

		timestamp2, err := svc.GetLatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp2.After(timestamp1), test.ShouldBeTrue)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})
}

func TestClose(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	t.Run("is idempotent and makes all endpoints return closed errors when feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "replay_sensor"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			MapRateSec:    &testMapRateSec,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		// call twice, assert result is the same to prove idempotence
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

		pose, componentRef, err := svc.GetPosition(ctx)
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentRef, test.ShouldBeEmpty)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)

		gpcmF, err := svc.GetPointCloudMap(ctx)
		test.That(t, gpcmF, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)

		gisF, err := svc.GetInternalState(ctx)
		test.That(t, gisF, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)

		mapTime, err := svc.GetLatestMapInfo(ctx)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)
		test.That(t, mapTime, test.ShouldResemble, time.Time{})

		cmd := map[string]interface{}{}
		resp, err := svc.DoCommand(ctx, cmd)
		test.That(t, resp, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)
	})
}

func TestDoCommand(t *testing.T) {
	logger := golog.NewTestLogger(t)

	termFunc := testhelper.InitTestCL(t, logger)
	defer termFunc()

	dataDirectory, err := os.MkdirTemp("", "*")
	test.That(t, err, test.ShouldBeNil)
	defer func() {
		err := os.RemoveAll(dataDirectory)
		test.That(t, err, test.ShouldBeNil)
	}()

	test.That(t, err, test.ShouldBeNil)
	attrCfg := &vcConfig.Config{
		Camera:        map[string]string{"name": "good_lidar", "data_frequency_hz": testDataFreqHz},
		ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
		DataDirectory: dataDirectory,
		MapRateSec:    &testMapRateSec,
	}
	svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
	test.That(t, err, test.ShouldBeNil)
	t.Run("returns UnimplementedError when given other parmeters", func(t *testing.T) {
		cmd := map[string]interface{}{"fake_flag": true}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldEqual, viamgrpc.UnimplementedError)
		test.That(t, resp, test.ShouldBeNil)
	})
	t.Run("returns UnimplementedError when given no parmeters", func(t *testing.T) {
		cmd := map[string]interface{}{}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldEqual, viamgrpc.UnimplementedError)
		test.That(t, resp, test.ShouldBeNil)
	})
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
}
