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
	"go.viam.com/rdk/referenceframe"
	spatial "go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	slamConfig "go.viam.com/slam/config"
	"go.viam.com/slam/sensors/lidar"
	slamTesthelper "go.viam.com/slam/testhelper"
	"go.viam.com/test"
	"go.viam.com/utils"

	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
)

func TestNew(t *testing.T) {
	logger := golog.NewTestLogger(t)
	dataDir, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Successful creation of cartographer slam service with no sensor", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		test.That(t, err, test.ShouldBeNil)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		svc, err := createSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with more than one sensor", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		test.That(t, err, test.ShouldBeNil)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"lidar", "one-too-many"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		svc, err := createSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: 'sensors' must contain only one "+
				"lidar camera, but is 'sensors: [lidar, one-too-many]'"))

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with non-existing sensor", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"gibberish"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			DataRateMsec:  validDataRateMsec,
			UseLiveData:   &_true,
		}

		_, err := createSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: error getting lidar camera "+
				"gibberish for slam service: \"gibberish\" missing from dependencies"))
	})

	t.Run("Successful creation of cartographer slam service with good lidar", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			DataRateMsec:  validDataRateMsec,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		svc, err := createSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeNil)

		grpcServer.Stop()
		test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)
	})

	t.Run("Failed creation of cartographer slam service with invalid sensor "+
		"that errors during call to NextPointCloud", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"invalid_sensor"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: dataDir,
			DataRateMsec:  validDataRateMsec,
			UseLiveData:   &_true,
		}

		_, err = createSLAMService(t, attrCfg, logger, false, testExecutableName)
		test.That(t, err, test.ShouldBeError,
			errors.New("runtime slam service error: error getting data from sensor: invalid sensor"))
	})

	testhelper.ClearDirectory(t, dataDir)
}

func TestDataProcess(t *testing.T) {
	logger, obs := golog.NewObservedTestLogger(t)
	dataDir, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &slamConfig.AttrConfig{
		Sensors:       []string{"good_lidar"},
		ConfigParams:  map[string]string{"mode": "2d"},
		DataDirectory: dataDir,
		DataRateMsec:  validDataRateMsec,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	svc, err := createSLAMService(t, attrCfg, logger, false, testExecutableName)
	test.That(t, err, test.ShouldBeNil)

	grpcServer.Stop()
	test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)

	slamSvc := svc.(testhelper.Service)

	t.Run("Successful startup of data process with good lidar", func(t *testing.T) {
		sensors := []string{"good_lidar"}
		lidar, err := lidar.New(context.Background(), testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		c := make(chan int, 100)
		slamSvc.StartDataProcess(cancelCtx, lidar, c)

		<-c
		cancelFunc()
		files, err := os.ReadDir(dataDir + "/data/")
		test.That(t, len(files), test.ShouldBeGreaterThanOrEqualTo, 1)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Failed startup of data process with invalid sensor "+
		"that errors during call to NextPointCloud", func(t *testing.T) {
		sensors := []string{"invalid_sensor"}
		lidar, err := lidar.New(context.Background(), testhelper.SetupDeps(sensors), sensors, 0)
		test.That(t, err, test.ShouldBeNil)

		cancelCtx, cancelFunc := context.WithCancel(context.Background())
		c := make(chan int, 100)
		slamSvc.StartDataProcess(cancelCtx, lidar, c)

		<-c
		allObs := obs.All()
		latestLoggedEntry := allObs[len(allObs)-1]
		cancelFunc()
		test.That(t, fmt.Sprint(latestLoggedEntry), test.ShouldContainSubstring, "invalid sensor")
	})

	test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)

	testhelper.ClearDirectory(t, dataDir)
}

func TestEndpointFailures(t *testing.T) {
	logger := golog.NewTestLogger(t)
	dataDir, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	grpcServer, port := setupTestGRPCServer(t)
	attrCfg := &slamConfig.AttrConfig{
		Sensors:       []string{"good_lidar"},
		ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
		DataDirectory: dataDir,
		MapRateSec:    &validMapRateMsec,
		DataRateMsec:  validDataRateMsec,
		Port:          "localhost:" + strconv.Itoa(port),
		UseLiveData:   &_true,
	}

	svc, err := createSLAMService(t, attrCfg, logger, false, testExecutableName)
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

	mimeType, im, pc, err := svc.GetMap(
		context.Background(), "hi", rdkutils.MimeTypePCD, cp, true, map[string]interface{}{})
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

	testhelper.ClearDirectory(t, dataDir)
}

func TestSLAMProcess(t *testing.T) {
	logger := golog.NewTestLogger(t)
	dataDir, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Successful start of live SLAM process with default parameters", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}

		svc, err := createSLAMService(t, attrCfg, logger, false, testExecutableName)
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
			{"-data_dir=" + dataDir},
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

	t.Run("Successful start of offline SLAM process with default parameters", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
			DataDirectory: dataDir,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_false,
		}

		svc, err := createSLAMService(t, attrCfg, logger, false, testExecutableName)
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
			{"-data_dir=" + dataDir},
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

	t.Run("Failed start of SLAM process that errors out due to invalid binary location", func(t *testing.T) {
		grpcServer, port := setupTestGRPCServer(t)
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d", "test_param": "viam"},
			DataDirectory: dataDir,
			MapRateSec:    &validMapRateMsec,
			DataRateMsec:  validDataRateMsec,
			Port:          "localhost:" + strconv.Itoa(port),
			UseLiveData:   &_true,
		}
		_, err := createSLAMService(t, attrCfg, logger, false, "hokus_pokus_does_not_exist_filename")
		test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "executable file not found in $PATH")
		grpcServer.Stop()
	})

	testhelper.ClearDirectory(t, dataDir)
}
