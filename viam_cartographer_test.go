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

	"github.com/pkg/errors"
	viamgrpc "go.viam.com/rdk/grpc"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	"github.com/viamrobotics/viam-cartographer/cartofacade"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	"github.com/viamrobotics/viam-cartographer/postprocess"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/testhelper"
)

const (
	testExecutableName  = "true" // the program "true", not the boolean value
	testDataFreqHz      = "5"
	testIMUDataFreqHz   = "20"
	testLidarDataFreqHz = "5"
)

var (
	_zeroTime       = time.Time{}
	_true           = true
	_false          = false
	emptyPointCloud = []byte{
		86, 69, 82, 83, 73, 79, 78, 32,
		46, 55, 10, 70, 73, 69, 76, 68,
		83, 32, 120, 32, 121, 32, 122,
		10, 83, 73, 90, 69, 32, 52, 32,
		52, 32, 52, 10, 84, 89, 80, 69,
		32, 70, 32, 70, 32, 70, 10, 67,
		79, 85, 78, 84, 32, 49, 32, 49,
		32, 49, 10, 87, 73, 68, 84, 72,
		32, 48, 10, 72, 69, 73, 71, 72,
		84, 32, 49, 10, 86, 73, 69, 87,
		80, 79, 73, 78, 84, 32, 48, 32,
		48, 32, 48, 32, 49, 32, 48, 32,
		48, 32, 48, 10, 80, 79, 73, 78,
		84, 83, 32, 48, 10, 68, 65, 84,
		65, 32, 98, 105, 110, 97, 114,
		121, 10,
	}
)

func TestNew(t *testing.T) {
	logger := logging.NewTestLogger(t)

	t.Run("Succeeds if use_cloud_slam is set to true. Causes all endpoints return errors.", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		attrCfg := &vcConfig.Config{
			Camera:       map[string]string{"name": string(s.GoodLidar), "data_frequency_hz": "5"},
			ConfigParams: map[string]string{"mode": "2d"},
			UseCloudSlam: &_true,
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

		prop, err := svc.Properties(ctx)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrUseCloudSlamEnabled)
		test.That(t, prop, test.ShouldResemble, slam.Properties{})

		cmd := map[string]interface{}{}
		resp, err := svc.DoCommand(ctx, cmd)
		test.That(t, resp, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrUseCloudSlamEnabled)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service with good lidar without IMU", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": string(s.GoodLidar), "data_frequency_hz": testLidarDataFreqHz},
			ConfigParams:  map[string]string{"mode": "2d"},
			EnableMapping: &_true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service with good lidar without IMU name specified", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		attrCfg := &vcConfig.Config{
			Camera:         map[string]string{"name": string(s.GoodLidar), "data_frequency_hz": testLidarDataFreqHz},
			MovementSensor: map[string]string{"name": ""},
			ConfigParams:   map[string]string{"mode": "2d"},
			EnableMapping:  &_true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service with good lidar with IMU", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		attrCfg := &vcConfig.Config{
			Camera:         map[string]string{"name": string(s.GoodLidar), "data_frequency_hz": testLidarDataFreqHz},
			ConfigParams:   map[string]string{"mode": "2d"},
			MovementSensor: map[string]string{"name": string(s.GoodIMU), "data_frequency_hz": testIMUDataFreqHz},
			EnableMapping:  &_true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		prop, err := svc.Properties(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, prop.CloudSlam, test.ShouldBeFalse)
		test.That(t, prop.MappingMode, test.ShouldEqual, slam.MappingModeNewMap)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service in localization mode", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		existingMap, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": string(s.GoodLidar)},
			ConfigParams:  map[string]string{"mode": "2d"},
			EnableMapping: &_false,
			ExistingMap:   existingMap,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		cs, ok := svc.(*viamcartographer.CartographerService)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, cs.SlamMode, test.ShouldEqual, cartofacade.LocalizingMode)

		timestamp1, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		// Test position
		pose, componentReference, err := svc.Position(context.Background())
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentReference, test.ShouldBeEmpty)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "VIAM_CARTO_GET_POSITION_NOT_INITIALIZED")

		// Test pointcloud map
		pcmFunc, err := svc.PointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcm, err := slam.HelperConcatenateChunksToFull(pcmFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcm, test.ShouldNotBeNil)

		// Test internal state
		isFunc, err := svc.InternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)

		is, err := slam.HelperConcatenateChunksToFull(isFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, is, test.ShouldNotBeNil)

		timestamp2, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp1, test.ShouldResemble, timestamp2)

		// Test properties
		prop, err := svc.Properties(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, prop.CloudSlam, test.ShouldBeFalse)
		test.That(t, prop.MappingMode, test.ShouldEqual, slam.MappingModeLocalizationOnly)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Successful creation of cartographer slam service in non localization mode", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		existingMap, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": string(s.GoodLidar)},
			ConfigParams:  map[string]string{"mode": "2d"},
			EnableMapping: &_true,
			ExistingMap:   existingMap,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeNil)

		cs, ok := svc.(*viamcartographer.CartographerService)
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, cs.SlamMode, test.ShouldEqual, cartofacade.UpdatingMode)

		// Test position
		pose, componentReference, err := svc.Position(context.Background())
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentReference, test.ShouldBeEmpty)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "VIAM_CARTO_GET_POSITION_NOT_INITIALIZED")

		timestamp1, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		// Test pointcloud map
		pcmFunc, err := svc.PointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)

		pcm, err := slam.HelperConcatenateChunksToFull(pcmFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcm, test.ShouldNotBeNil)

		// Test internal state
		isFunc, err := svc.InternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)

		is, err := slam.HelperConcatenateChunksToFull(isFunc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, is, test.ShouldNotBeNil)

		timestamp2, err := svc.LatestMapInfo(context.Background())
		test.That(t, err, test.ShouldBeNil)

		test.That(t, timestamp1.After(_zeroTime), test.ShouldBeTrue)
		test.That(t, timestamp2.After(timestamp1), test.ShouldBeTrue)

		// Test properties
		prop, err := svc.Properties(context.Background())
		test.That(t, err, test.ShouldBeNil)
		test.That(t, prop.CloudSlam, test.ShouldBeFalse)
		test.That(t, prop.MappingMode, test.ShouldEqual, slam.MappingModeUpdateExistingMap)

		test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with good lidar with movement"+
		"sensor that does not support IMU nor odometer", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		attrCfg := &vcConfig.Config{
			Camera:         map[string]string{"name": string(s.GoodLidar), "data_frequency_hz": testLidarDataFreqHz},
			MovementSensor: map[string]string{"name": string(s.MovementSensorNotIMUNotOdometer)},
			ConfigParams:   map[string]string{"mode": "2d"},
			EnableMapping:  &_true,
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeError, s.ErrMovementSensorNeitherIMUNorOdometer)
		test.That(t, svc, test.ShouldBeNil)
	})

	t.Run("Fails to create cartographer slam service with no sensor", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		_, fsCleanupFunc := testhelper.InitInternalState(t)
		defer fsCleanupFunc()

		attrCfg := &vcConfig.Config{
			Camera:       map[string]string{},
			ConfigParams: map[string]string{"mode": "2d"},
		}

		svc, err := testhelper.CreateSLAMService(t, attrCfg, logger)
		test.That(t, err, test.ShouldBeError, errors.New("error validating \"path\": \"camera[name]\" is required"))
		test.That(t, svc, test.ShouldBeNil)
	})
}

func TestClose(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	t.Run("is idempotent and makes all endpoints return closed errors", func(t *testing.T) {
		termFunc := testhelper.InitTestCL(t, logger)
		defer termFunc()

		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		attrCfg := &vcConfig.Config{
			Camera:        map[string]string{"name": string(s.ReplayLidar)},
			ConfigParams:  map[string]string{"mode": "2d"},
			EnableMapping: &_true,
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

		prop, err := svc.Properties(ctx)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)
		test.That(t, prop, test.ShouldResemble, slam.Properties{})

		cmd := map[string]interface{}{}
		resp, err := svc.DoCommand(ctx, cmd)
		test.That(t, resp, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, viamcartographer.ErrClosed)
	})
}

func TestDoCommand(t *testing.T) {
	logger := logging.NewTestLogger(t)

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
		Camera:        map[string]string{"name": string(s.GoodLidar), "data_frequency_hz": testLidarDataFreqHz},
		ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
		EnableMapping: &_true,
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
	t.Run("returns false when given 'job_done'", func(t *testing.T) {
		cmd := map[string]interface{}{viamcartographer.JobDoneCommand: ""}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldBeNil)
		test.That(
			t,
			resp, test.ShouldResemble,
			map[string]interface{}{viamcartographer.JobDoneCommand: false},
		)
	})
	t.Run("changes postprocess bool after 'postprocess_toggle'", func(t *testing.T) {
		cmd := map[string]interface{}{postprocess.ToggleCommand: ""}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldBeNil)
		test.That(
			t,
			resp, test.ShouldResemble,
			map[string]interface{}{viamcartographer.PostprocessToggleResponseKey: true},
		)

		cmd = map[string]interface{}{postprocess.ToggleCommand: ""}
		resp, err = svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldBeNil)
		test.That(
			t,
			resp, test.ShouldResemble,
			map[string]interface{}{viamcartographer.PostprocessToggleResponseKey: false},
		)
	})
	t.Run(
		"errors if 'postprocess_undo' is called before any postprocessing has occurred",
		func(t *testing.T) {
			cmd := map[string]interface{}{postprocess.UndoCommand: ""}
			resp, err := svc.DoCommand(context.Background(), cmd)
			test.That(t, err, test.ShouldBeError, viamcartographer.ErrNoPostprocessingToUndo)
			test.That(t, resp, test.ShouldBeNil)
		})
	t.Run(
		"succeeds if 'postprocess_undo' is called after any postprocessing has occurred",
		func(t *testing.T) {
			point := map[string]interface{}{"X": float64(1), "Y": float64(1)}
			cmd := map[string]interface{}{postprocess.AddCommand: []interface{}{point}}
			resp, err := svc.DoCommand(context.Background(), cmd)
			test.That(t, err, test.ShouldBeNil)
			test.That(
				t,
				resp,
				test.ShouldResemble,
				map[string]interface{}{postprocess.AddCommand: viamcartographer.SuccessMessage},
			)

			cmd = map[string]interface{}{postprocess.UndoCommand: ""}
			resp, err = svc.DoCommand(context.Background(), cmd)
			test.That(t, err, test.ShouldBeNil)
			test.That(
				t,
				resp,
				test.ShouldResemble,
				map[string]interface{}{postprocess.UndoCommand: viamcartographer.SuccessMessage},
			)
		})
	t.Run(
		"success if 'postprocess_add' is called correctly",
		func(t *testing.T) {
			point := map[string]interface{}{"X": float64(1), "Y": float64(1)}
			cmd := map[string]interface{}{postprocess.AddCommand: []interface{}{point}}
			resp, err := svc.DoCommand(context.Background(), cmd)
			test.That(t, err, test.ShouldBeNil)
			test.That(
				t,
				resp,
				test.ShouldResemble,
				map[string]interface{}{postprocess.AddCommand: viamcartographer.SuccessMessage},
			)
		})
	t.Run(
		"errors if 'postprocess_add' is called with incorrect format",
		func(t *testing.T) {
			cmd := map[string]interface{}{postprocess.AddCommand: "hello"}
			resp, err := svc.DoCommand(context.Background(), cmd)
			test.That(t, err.Error(), test.ShouldContainSubstring, viamcartographer.ErrBadPostprocessingPointsFormat.Error())
			test.That(t, resp, test.ShouldBeNil)
		})
	t.Run(
		"success if 'postprocess_remove' is called correctly",
		func(t *testing.T) {
			point := map[string]interface{}{"X": float64(1), "Y": float64(1)}
			cmd := map[string]interface{}{postprocess.RemoveCommand: []interface{}{point}}
			resp, err := svc.DoCommand(context.Background(), cmd)
			test.That(t, err, test.ShouldBeNil)
			test.That(
				t,
				resp,
				test.ShouldResemble,
				map[string]interface{}{postprocess.RemoveCommand: viamcartographer.SuccessMessage},
			)
		})
	t.Run(
		"errors if 'postprocess_remove' is called with incorrect format",
		func(t *testing.T) {
			cmd := map[string]interface{}{postprocess.RemoveCommand: "hello"}
			resp, err := svc.DoCommand(context.Background(), cmd)
			test.That(t, err.Error(), test.ShouldContainSubstring, viamcartographer.ErrBadPostprocessingPointsFormat.Error())
			test.That(t, resp, test.ShouldBeNil)
		})
	t.Run(
		"success if 'postprocess_path' is called correctly",
		func(t *testing.T) {
			path := artifact.MustPath("viam-cartographer/outputs/viam-office-02-22-3/pointcloud/pointcloud_1.pcd")
			cmd := map[string]interface{}{postprocess.PathCommand: path}
			resp, err := svc.DoCommand(context.Background(), cmd)
			test.That(t, err, test.ShouldBeNil)
			test.That(
				t,
				resp,
				test.ShouldResemble,
				map[string]interface{}{postprocess.PathCommand: viamcartographer.SuccessMessage},
			)
		})
	t.Run(
		"errors if 'postprocess_path' is called with incorrect format",
		func(t *testing.T) {
			point := map[string]interface{}{"X": float64(1), "Y": float64(1)}
			cmd := map[string]interface{}{postprocess.PathCommand: point}
			resp, err := svc.DoCommand(context.Background(), cmd)
			test.That(t, err.Error(), test.ShouldContainSubstring, viamcartographer.ErrBadPostprocessingPath.Error())
			test.That(t, resp, test.ShouldBeNil)
		})
	t.Run("position history value and enable return expectedly", func(t *testing.T) {
		cmd := map[string]interface{}{viamcartographer.PositionHistoryValueCommand: ""}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldBeNil)
		test.That(
			t,
			resp, test.ShouldResemble,
			map[string]interface{}{viamcartographer.PositionHistoryValueCommand: false},
		)

		cmd = map[string]interface{}{viamcartographer.PositionHistoryEnableCommand: ""}
		resp, err = svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldBeNil)
		test.That(
			t,
			resp, test.ShouldResemble,
			map[string]interface{}{viamcartographer.PositionHistoryEnableCommand: viamcartographer.SuccessMessage},
		)

		cmd = map[string]interface{}{viamcartographer.PositionHistoryValueCommand: ""}
		resp, err = svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldBeNil)
		test.That(
			t,
			resp, test.ShouldResemble,
			map[string]interface{}{viamcartographer.PositionHistoryValueCommand: true},
		)
	})
	t.Run("position history get returns a byte slice of the pointcloud", func(t *testing.T) {
		cmd := map[string]interface{}{viamcartographer.PositionHistoryGetCommand: ""}
		resp, err := svc.DoCommand(context.Background(), cmd)
		test.That(t, err, test.ShouldBeNil)
		val, ok := resp[viamcartographer.PositionHistoryGetCommand]
		test.That(t, ok, test.ShouldBeTrue)
		test.That(t, val, test.ShouldResemble, emptyPointCloud)
	})
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
}
