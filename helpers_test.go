// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user.
package viamcartographer_test

import (
	"context"
	"net"
	"os"
	"strconv"
	"sync/atomic"
	"testing"

	"github.com/edaniels/golog"
	"github.com/edaniels/gostream"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/testutils/inject"
	slamConfig "go.viam.com/slam/config"
	slamTesthelper "go.viam.com/slam/testhelper"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"
	"google.golang.org/grpc"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
)

const (
	testExecutableName         = "true"
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
		case "invalid_sensor":
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
	executableName string,
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

	svc, err := viamcartographer.New(ctx, deps, cfgService, logger, bufferSLAMProcessLogs, executableName)

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
