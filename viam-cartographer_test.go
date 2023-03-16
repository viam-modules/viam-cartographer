// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user.
package viamcartographer_test

import (
	"context"
	"fmt"
	"math"
	"net"
	"os"
	"strconv"
	"sync/atomic"
	"testing"

	"github.com/edaniels/golog"
	"github.com/edaniels/gostream"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/slam"
	spatial "go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	rdkutils "go.viam.com/rdk/utils"
	slamConfig "go.viam.com/slam/config"
	slamTesthelper "go.viam.com/slam/testhelper"
	"go.viam.com/test"
	"go.viam.com/utils"
	"go.viam.com/utils/artifact"
	"google.golang.org/grpc"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
)

const (
	validDataRateMS            = 200
	numCartographerPointClouds = 15
	dataBufferSize             = 4
)

var (
	cartographerIntLidarReleasePointCloudChan = make(chan int, 1)
	validMapRate                              = 200
	_true                                     = true
	_false                                    = false
)

func closeOutSLAMService(t *testing.T, name string) {
	t.Helper()

	if name != "" {
		err := slamTesthelper.ResetFolder(name)
		test.That(t, err, test.ShouldBeNil)
	}
}

func setupTestGRPCServer(tb testing.TB) (*grpc.Server, int) {
	listener, err := net.Listen("tcp", ":0")
	test.That(tb, err, test.ShouldBeNil)
	grpcServer := grpc.NewServer()
	go grpcServer.Serve(listener)

	return grpcServer, listener.Addr().(*net.TCPAddr).Port
}

func setBinaryLocationForTesting(val string) {
	viamcartographer.BinaryLocation = val
}

func setupDeps(attr *slamConfig.AttrConfig) registry.Dependencies {
	deps := make(registry.Dependencies)

	for _, sensor := range attr.Sensors {
		cam := &inject.Camera{}
		switch sensor {
		case "good_lidar":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return pointcloud.New(), nil
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return nil, errors.New("lidar not camera")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, nil
			}
			deps[camera.Named(sensor)] = cam
		case "bad_lidar":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("bad_lidar")
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return nil, errors.New("lidar not camera")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			deps[camera.Named(sensor)] = cam
		case "good_camera":
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				return nil, errors.New("camera not lidar")
			}
			deps[camera.Named(sensor)] = cam
		case "gibberish":
			return deps
		case "cartographer_int_lidar":
			var index uint64
			cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
				select {
				case <-cartographerIntLidarReleasePointCloudChan:
					i := atomic.AddUint64(&index, 1) - 1
					if i >= numCartographerPointClouds {
						return nil, errors.New("No more cartographer point clouds")
					}
					file, err := os.Open(artifact.MustPath("slam/mock_lidar/" + strconv.FormatUint(i, 10) + ".pcd"))
					if err != nil {
						return nil, err
					}
					pointCloud, err := pointcloud.ReadPCD(file)
					if err != nil {
						return nil, err
					}
					return pointCloud, nil
				default:
					return nil, errors.Errorf("Lidar not ready to return point cloud %v", index)
				}
			}
			cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
				return nil, errors.New("lidar not camera")
			}
			cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
				return nil, transform.NewNoIntrinsicsError("")
			}
			cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
				return camera.Properties{}, nil
			}
			deps[camera.Named(sensor)] = cam
		default:
			continue
		}
	}
	return deps
}

func createSLAMService(
	t *testing.T,
	attrCfg *slamConfig.AttrConfig,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
	success bool,
) (slam.Service, error) {
	t.Helper()

	ctx := context.Background()
	cfgService := config.Service{Name: "test", Type: "slam", Model: viamcartographer.Model}
	cfgService.ConvertedAttributes = attrCfg

	deps := setupDeps(attrCfg)

	sensorDeps, err := attrCfg.Validate("path")
	if err != nil {
		return nil, err
	}
	test.That(t, sensorDeps, test.ShouldResemble, attrCfg.Sensors)

	viamcartographer.SetCameraValidationMaxTimeoutSecForTesting(1)
	viamcartographer.SetDialMaxTimeoutSecForTesting(1)

	svc, err := viamcartographer.New(ctx, deps, cfgService, logger, bufferSLAMProcessLogs)

	if success {
		if err != nil {
			return nil, err
		}
		test.That(t, svc, test.ShouldNotBeNil)
		return svc, nil
	}

	test.That(t, svc, test.ShouldBeNil)
	return nil, err
}

func TestGeneralNew(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	originalBinaryLocation := viamcartographer.BinaryLocation
	setBinaryLocationForTesting("true")
	defer setBinaryLocationForTesting(originalBinaryLocation)

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

		svc, err := createSLAMService(t, attrCfg, logger, false, true)
		test.That(t, err, test.ShouldBeNil)

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

		_, err := createSLAMService(t, attrCfg, logger, false, false)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring camera error: error getting camera gibberish for slam service: \"gibberish\" missing from dependencies"))
	})

	closeOutSLAMService(t, name)
}

func TestCartographerNew(t *testing.T) {
	logger := golog.NewTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	originalBinaryLocation := viamcartographer.BinaryLocation
	setBinaryLocationForTesting("true")
	defer setBinaryLocationForTesting(originalBinaryLocation)

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

		svc, err := createSLAMService(t, attrCfg, logger, false, true)
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

		_, err = createSLAMService(t, attrCfg, logger, false, false)
		test.That(t, err, test.ShouldBeError,
			errors.Errorf("runtime slam service error: error getting data from sensor: %v", attrCfg.Sensors[0]))
	})

	t.Run("New cartographer service with camera without NextPointCloud implementation", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_camera"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: name,
			DataRateMsec:  validDataRateMS,
			UseLiveData:   &_true,
		}

		_, err = createSLAMService(t, attrCfg, logger, false, false)
		test.That(t, err, test.ShouldBeError,
			errors.New("runtime slam service error: error getting data from sensor: camera not lidar"))
	})
	closeOutSLAMService(t, name)
}

func TestCartographerDataProcess(t *testing.T) {
	logger, obs := golog.NewObservedTestLogger(t)
	name, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	originalBinaryLocation := viamcartographer.BinaryLocation
	setBinaryLocationForTesting("true")
	defer setBinaryLocationForTesting(originalBinaryLocation)

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
	svc, err := createSLAMService(t, attrCfg, logger, false, true)
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

	originalBinaryLocation := viamcartographer.BinaryLocation
	setBinaryLocationForTesting("true")
	defer setBinaryLocationForTesting(originalBinaryLocation)

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
	svc, err := createSLAMService(t, attrCfg, logger, false, true)
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

	originalBinaryLocation := viamcartographer.BinaryLocation
	setBinaryLocationForTesting("true")
	defer setBinaryLocationForTesting(originalBinaryLocation)

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
		svc, err := createSLAMService(t, attrCfg, logger, false, true)
		test.That(t, err, test.ShouldBeNil)

		slamSvc := svc.(testhelper.Service)
		processCfg := slamSvc.GetSLAMProcessConfig()
		cmd := append([]string{processCfg.Name}, processCfg.Args...)

		cmdResult := [][]string{
			{viamcartographer.BinaryLocation},
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
		svc, err := createSLAMService(t, attrCfg, logger, false, true)
		test.That(t, err, test.ShouldBeNil)

		slamSvc := svc.(testhelper.Service)
		processCfg := slamSvc.GetSLAMProcessConfig()
		cmd := append([]string{processCfg.Name}, processCfg.Args...)

		cmdResult := [][]string{
			{viamcartographer.BinaryLocation},
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

	originalBinaryLocation := viamcartographer.BinaryLocation
	setBinaryLocationForTesting("true")
	defer setBinaryLocationForTesting(originalBinaryLocation)

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
	svc, err := createSLAMService(t, attrCfg, logger, false, true)
	test.That(t, err, test.ShouldBeNil)

	slamSvc := svc.(testhelper.Service)

	t.Run("Run SLAM process that errors out due to invalid binary location", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		delete(slam.SLAMLibraries, "fake_cartographer")
		originalBinaryLocation := viamcartographer.BinaryLocation
		// This test ensures that we get the correct error if the user does
		// not have the correct binary installed. This sets the binary path to some
		// unused value and then tests to ensure that creating a SLAM process fails
		setBinaryLocationForTesting("fail_this_binary_does_not_exist")
		defer setBinaryLocationForTesting(originalBinaryLocation)

		err := slamSvc.StartSLAMProcess(cancelCtx)
		test.That(t, fmt.Sprint(err), test.ShouldContainSubstring, "problem adding slam process:")

		cancelFunc()

		err = slamSvc.StopSLAMProcess()
		test.That(t, err, test.ShouldBeNil)
	})

	grpcServer.Stop()
	test.That(t, utils.TryClose(context.Background(), svc), test.ShouldBeNil)

	closeOutSLAMService(t, name)
}
