// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user.
package viamcartographer_test

import (
	"context"
	"fmt"
	"math"
	"os"
	"strconv"
	"testing"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	spatial "go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	rdkutils "go.viam.com/rdk/utils"
	slamConfig "go.viam.com/slam/config"
	slamTesthelper "go.viam.com/slam/testhelper"
	"go.viam.com/test"
	"go.viam.com/utils"

	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
)

func TestGeneralNew(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("New slam service with no camera", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		test.That(t, err, test.ShouldBeNil)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: name,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("New slam service with more than one camera", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		test.That(t, err, test.ShouldBeNil)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"lidar", "one-too-many"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: name,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring camera error: 'sensors' must contain only one lidar camera, but is 'sensors: [lidar, one-too-many]'"))

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("New slam service with bad camera", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"gibberish"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		_, err := createSLAMService(t, attrCfg, logger, false, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring camera error: error getting camera gibberish for slam service: \"gibberish\" missing from dependencies"))
	})

	closeOutSLAMService(t, name)
}

func TestCartographerNew(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("New cartographer service with good lidar in slam mode 2d", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("New cartographer service with lidar that errors during call to NextPointCloud", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"bad_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		_, err = createSLAMService(t, attrCfg, logger, false, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.Errorf("runtime slam service error: error getting data from sensor: %v", attrCfg.Sensors[0]))
	})

	t.Run("New cartographer service with camera without NextPointCloud implementation", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"invalid_sensor"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		_, err = createSLAMService(t, attrCfg, logger, false, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("runtime slam service error: error getting data from sensor: camera not lidar"))
	})
	closeOutSLAMService(t, name)
}

func TestCartographerDataProcess(t *testing.T) {
	logger, obs := golog.NewObservedTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &slamConfig.AttrConfig{
		Sensors:       []string{"good_lidar"},
		ConfigParams:  map[string]string{"mode": "2d"},
		DataDirectory: name,
		DataRateMsec:  validDataRateMS,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	// Create slam service
	svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
	test.That(t, err, test.ShouldBeNil)

	grpcServer.Stop()
	test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)

	slamSvc := svc.(testhelper.Service)

	t.Run("Cartographer Data Process with lidar in slam mode 2d", func(t *testing.T) {
		goodCam := &inject.Camera{}
		goodCam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
			return pointcloud.New(), nil
		}
		goodCam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
			return camera.Properties{}, nil
		}
		cams := []camera.Camera{goodCam}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		c := make(chan int, 100)
		slamSvc.StartDataProcess(cancelCtx, cams, c)

		<-c
		cancelFunc()
		files, err := os.ReadDir(name + "/data/")
		test.That(t, len(files), test.ShouldBeGreaterThanOrEqualTo, 1)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Cartographer Data Process with lidar that errors during call to NextPointCloud", func(t *testing.T) {
		badCam := &inject.Camera{}
		badCam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
			return nil, errors.New("bad_lidar")
		}
		badCam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
			return camera.Properties{}, nil
		}
		cams := []camera.Camera{badCam}

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		c := make(chan int, 100)
		slamSvc.StartDataProcess(cancelCtx, cams, c)

		<-c
		allObs := obs.All()
		latestLoggedEntry := allObs[len(allObs)-1]
		cancelFunc()
		test.That(t, fmt.Sprint(latestLoggedEntry), test.ShouldContainSubstring, "bad_lidar")
	})

	test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)

	closeOutSLAMService(t, name)
}

func TestEndpointFailures(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &slamConfig.AttrConfig{
		Sensors:       []string{"good_lidar"},
		ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
		DataDirectory: name,
		MapRateSec:    &validMapRate,
		DataRateMsec:  validDataRateMS,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	// Create slam service
	svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
	test.That(t, err, test.ShouldBeNil)

	p, err := svc.Position(context.Background(), "hi", map[string]interface{}{})
	test.That(t, p, test.ShouldBeNil)
	test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "error getting SLAM position")

	pNew, frame, err := svc.GetPosition(context.Background(), "hi")
	test.That(t, pNew, test.ShouldBeNil)
	test.That(t, frame, test.ShouldBeEmpty)
	test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "error getting SLAM position")

	pose := spatial.NewPose(r3.Vector{X: 1, Y: 2, Z: 3},
		&spatial.OrientationVector{Theta: math.Pi / 2, OX: 0, OY: 0, OZ: -1})
	cp := referenceframe.NewPoseInFrame("frame", pose)

	mimeType, im, pc, err := svc.GetMap(context.Background(), "hi", rdkutils.MimeTypePCD, cp, true, map[string]interface{}{})
	test.That(t, mimeType, test.ShouldResemble, "")
	test.That(t, im, test.ShouldBeNil)
	test.That(t, pc, test.ShouldBeNil)
	test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "error getting SLAM map")

	internalState, err := svc.GetInternalState(context.Background(), "hi")
	test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "error getting the internal state from the SLAM client")
	test.That(t, internalState, test.ShouldBeNil)

	callbackPointCloud, err := svc.GetPointCloudMapStream(context.Background(), "hi")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, callbackPointCloud, test.ShouldNotBeNil)
	chunkPCD, err := callbackPointCloud()
	test.That(t, err.Error(), test.ShouldContainSubstring, "error receiving pointcloud chunk")
	test.That(t, chunkPCD, test.ShouldBeNil)

	callbackInternalState, err := svc.GetInternalStateStream(context.Background(), "hi")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, callbackInternalState, test.ShouldNotBeNil)
	chunkInternalState, err := callbackInternalState()
	test.That(t, err.Error(), test.ShouldContainSubstring, "error receiving internal state chunk")
	test.That(t, chunkInternalState, test.ShouldBeNil)

	grpcServer.Stop()
	test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)

	closeOutSLAMService(t, name)
}

func TestSLAMProcessSuccess(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Test online SLAM process with default parameters", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
			DataDirectory: name,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		// Create slam service
		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		slamSvc := svc.(testhelper.Service)
		processCfg := slamSvc.GetSLAMProcessConfig()
		cmd := append([]string{processCfg.Name}, processCfg.Args...)

		cmdResult := [][]string{
			{testExecutableName},
			{"-sensors=good_lidar"},
			{"-config_param={test_param=viam,mode=2d}", "-config_param={mode=2d,test_param=viam}"},
			{"-data_rate_ms=200"},
			{"-map_rate_sec=60"},
			{"-data_dir=" + name},
			{"-delete_processed_data=true"},
			{"-use_live_data=true"},
			{"-port=localhost:" + strconv.Itoa(port)},
			{"--aix-auto-update"},
		}

		for i, s := range cmd {
			t.Run(fmt.Sprintf("Test command argument %v at index %v", s, i), func(t *testing.T) {
				test.That(t, s, test.ShouldBeIn, cmdResult[i])
			})
		}

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("Test offline SLAM process with default parameters", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
			DataDirectory: name,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		// Create slam service
		svc, err := createSLAMService(t, attrCfg, logger, false, true, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		slamSvc := svc.(testhelper.Service)
		processCfg := slamSvc.GetSLAMProcessConfig()
		cmd := append([]string{processCfg.Name}, processCfg.Args...)

		cmdResult := [][]string{
			{testExecutableName},
			{"-sensors="},
			{"-config_param={mode=2d,test_param=viam}", "-config_param={test_param=viam,mode=2d}"},
			{"-data_rate_ms=200"},
			{"-map_rate_sec=60"},
			{"-data_dir=" + name},
			{"-delete_processed_data=false"},
			{"-use_live_data=false"},
			{"-port=localhost:" + strconv.Itoa(port)},
			{"--aix-auto-update"},
		}

		for i, s := range cmd {
			t.Run(fmt.Sprintf("Test command argument %v at index %v", s, i), func(t *testing.T) {
				test.That(t, s, test.ShouldBeIn, cmdResult[i])
			})
		}

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	closeOutSLAMService(t, name)
}

func TestSLAMProcessFail(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &slamConfig.AttrConfig{
		Sensors:       []string{"good_lidar"},
		ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
		DataDirectory: name,
		MapRateSec:    &validMapRate,
		DataRateMsec:  validDataRateMS,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	t.Run("Run SLAM process that errors out due to invalid binary location", func(t *testing.T) {
		_, err := createSLAMService(t, attrCfg, logger, false, true, "hokus_pokus_does_not_exist_filename")
		test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "executable file not found in $PATH")
	})

	grpcServer.Stop()
	closeOutSLAMService(t, name)
}
