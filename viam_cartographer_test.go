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
	testExecutableName  = "true" // the program "true", not the boolean value
	testDataFreqHz      = "5"
	testIMUDataFreqHz   = "20"
	testLidarDataFreqHz = "5"
)

var (
	testMapRateSec   = 200
	_zeroInt         = 0
	_zeroTime        = time.Time{}
	testDataRateMsec = 200
	_true            = true
)

func TestNew(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("Succeeds if use_cloud_slam is set to true. Causes all endpoints return errors.", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()
		attrCfg := &vcConfig.Config{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			DataRateMsec:  &testDataRateMsec,
			UseCloudSlam:  &_true,
		}

		ctx := context.Background()

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		pose, componentRef, err := svc.Position(ctx)
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentRef, test.ShouldBeEmpty)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrUseCloudSlamEnabled)

		gpcmF, err := svc.PointCloudMap(ctx)
		test.That(t, gpcmF, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrUseCloudSlamEnabled)

		gisF, err := svc.InternalState(ctx)
		test.That(t, gisF, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrUseCloudSlamEnabled)

		mapTime, err := svc.LatestMapInfo(ctx)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrUseCloudSlamEnabled)
		test.That(t, mapTime, test.ShouldResemble, time.Time{})

		cmd := map[string]interface{}{}
		resp, err := svc.DoCommand(ctx, cmd)
		test.That(t, resp, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrUseCloudSlamEnabled)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with more than one sensor", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()
		attrCfg := &vcConfig.Config{
			Sensors:       []string{"lidar", "one-too-many"},
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
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Sensors:       []string{"gibberish"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			DataRateMsec:  &testDataRateMsec,
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
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			DataRateMsec:  &testDataRateMsec,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with invalid sensor "+
		"that errors during call to Properties", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Sensors:       []string{"lidar_with_invalid_properties"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			DataRateMsec:  &testDataRateMsec,
		}

		_, err = testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,

			errors.New("configuring lidar camera error: 'camera' must support PCD"))
	})

	t.Run("Failed creation of cartographer slam service with bad sensor "+
		"that errors during call to NextPointCloud", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Sensors:       []string{"lidar_with_erroring_functions"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			DataRateMsec:  &testDataRateMsec,
		}

		_, err = testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,

			errors.New("failed to get data from lidar: ValidateGetLidarData timeout: NextPointCloud error: invalid sensor"))
	})

	t.Run("Successful creation of cartographer slam service in localization mode", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Sensors:       []string{"replay_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			MapRateSec:    &_zeroInt,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		cs, ok := svc.(*viamcartographer.CartographerService)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, cs.SlamMode, test.ShouldEqual, cartofacade.LocalizingMode)

		timestamp1, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pose, componentReference, err := svc.Position(context.Background())
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentReference, test.ShouldBeEmpty)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "VIAM_CARTO_GET_POSITION_NOT_INITIALIZED")

		pcmFunc, err := svc.PointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcm, err := slam.HelperConcatenateChunksToFull(pcmFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcm, test.ShouldNotBeNil)

		isFunc, err := svc.InternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)

		is, err := slam.HelperConcatenateChunksToFull(isFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, is, test.ShouldNotBeNil)

		timestamp2, err := svc.LatestMapInfo(context.Background())
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
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			DataRateMsec:  &testDataRateMsec,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		cs, ok := svc.(*viamcartographer.CartographerService)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, cs.SlamMode, test.ShouldEqual, cartofacade.UpdatingMode)

		pose, componentReference, err := svc.Position(context.Background())
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentReference, test.ShouldBeEmpty)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "VIAM_CARTO_GET_POSITION_NOT_INITIALIZED")

		pcmFunc, err := svc.PointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcm, err := slam.HelperConcatenateChunksToFull(pcmFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcm, test.ShouldNotBeNil)

		isFunc, err := svc.InternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)

		is, err := slam.HelperConcatenateChunksToFull(isFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, is, test.ShouldNotBeNil)

		timestamp1, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		timestamp2, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp2.After(timestamp1), test.ShouldBeTrue)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Fails to create cartographer slam service with no sensor", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError, errors.New("error validating \"path\": \"sensors\" must not be empty"))
		test.That(t, svc, test.ShouldBeNil)
	})
}

func TestNewFeatureFlag(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("Successful creation of cartographer slam service with good lidar with feature flag enabled, without IMU", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "good_lidar", "data_frequency_hz": testLidarDataFreqHz},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			NewConfigFlag: true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service with good lidar with feature flag enabled, with IMU", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Camera:         map[string]string{"name": "good_lidar", "data_frequency_hz": testLidarDataFreqHz},
			ConfigParams:   map[string]string{"mode": "2d"},
			DataDirectory:  dataDirectory,
			NewConfigFlag:  true,
			MovementSensor: map[string]string{"name": "good_imu", "data_frequency_hz": testIMUDataFreqHz},
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with invalid lidar sensor "+
		"that errors during call to NextPointCloud with feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "lidar_with_erroring_functions", "data_frequency_hz": testLidarDataFreqHz},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			NewConfigFlag: true,
		}

		_, err = testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,

			errors.New("failed to get data from lidar: ValidateGetLidarData timeout: NextPointCloud error: invalid sensor"))
	})

	t.Run("Failed creation of cartographer slam service with invalid IMU sensor "+
		"that errors during call to LinearAcceleration with feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Camera:         map[string]string{"name": "good_lidar", "data_frequency_hz": testLidarDataFreqHz},
			ConfigParams:   map[string]string{"mode": "2d"},
			DataDirectory:  dataDirectory,
			NewConfigFlag:  true,
			MovementSensor: map[string]string{"name": "imu_with_invalid_properties", "data_frequency_hz": testIMUDataFreqHz},
		}

		_, err = testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError,

			errors.New("configuring IMU movement sensor error: 'movement_sensor' must support both LinearAcceleration and AngularVelocity"))
	})

	t.Run("Successful creation of cartographer slam service in localization mode with feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			MapRateSec:    &_zeroInt,
			NewConfigFlag: true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		cs, ok := svc.(*viamcartographer.CartographerService)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, cs.SlamMode, test.ShouldEqual, cartofacade.LocalizingMode)

		timestamp1, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pose, componentReference, err := svc.Position(context.Background())
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentReference, test.ShouldBeEmpty)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "VIAM_CARTO_GET_POSITION_NOT_INITIALIZED")

		pcmFunc, err := svc.PointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcm, err := slam.HelperConcatenateChunksToFull(pcmFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcm, test.ShouldNotBeNil)

		isFunc, err := svc.InternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)

		is, err := slam.HelperConcatenateChunksToFull(isFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, is, test.ShouldNotBeNil)

		timestamp2, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp1, test.ShouldResemble, timestamp2)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service in non localization mode with feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			NewConfigFlag: true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		cs, ok := svc.(*viamcartographer.CartographerService)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, cs.SlamMode, test.ShouldEqual, cartofacade.UpdatingMode)

		pose, componentReference, err := svc.Position(context.Background())
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentReference, test.ShouldBeEmpty)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "VIAM_CARTO_GET_POSITION_NOT_INITIALIZED")

		timestamp1, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcmFunc, err := svc.PointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcm, err := slam.HelperConcatenateChunksToFull(pcmFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcm, test.ShouldNotBeNil)

		isFunc, err := svc.InternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)

		is, err := slam.HelperConcatenateChunksToFull(isFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, is, test.ShouldNotBeNil)

		timestamp2, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp2.After(timestamp1), test.ShouldBeTrue)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Fails to create cartographer slam service with no sensor with feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			NewConfigFlag: true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError, errors.New("error validating \"path\": \"camera[name]\" is required"))
		test.That(t, svc, test.ShouldBeNil)
	})
}

func TestClose(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	t.Run("is idempotent and makes all endpoints return closed errors with feature flag enabled", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": "replay_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDirectory,
			MapRateSec:    &testMapRateSec,
			NewConfigFlag: true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		// call twice, assert result is the same to prove idempotence
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

		pose, componentRef, err := svc.Position(ctx)
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentRef, test.ShouldBeEmpty)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)

		gpcmF, err := svc.PointCloudMap(ctx)
		test.That(t, gpcmF, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)

		gisF, err := svc.InternalState(ctx)
		test.That(t, gisF, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)

		mapTime, err := svc.LatestMapInfo(ctx)
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
		Camera:        map[string]string{"name": "good_lidar", "data_frequency_hz": testLidarDataFreqHz},
		ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
		DataDirectory: dataDirectory,
		MapRateSec:    &testMapRateSec,
		NewConfigFlag: true,
	}
	svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
	test.That(t, err, test.ShouldBeNil)
	t.Run("returns UnimplementedError when given other parameters", func(t *testing.T) {
		cmd := map[string]interface{}{"fake_flag": true}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldEqual, viamgrpc.UnimplementedError)
		test.That(t, resp, test.ShouldBeNil)
	})
	t.Run("returns UnimplementedError when given no parameters", func(t *testing.T) {
		cmd := map[string]interface{}{}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldEqual, viamgrpc.UnimplementedError)
		test.That(t, resp, test.ShouldBeNil)
	})
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
}
