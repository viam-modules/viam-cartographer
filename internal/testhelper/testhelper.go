// Package testhelper implements a slam service definition with additional exported functions for
// the purpose of testing
package testhelper

import (
	"bufio"
	"context"
	"os"
	"strconv"
	"sync/atomic"
	"testing"

	"github.com/edaniels/gostream"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/slam/sensors/lidar"
	slamTesthelper "go.viam.com/slam/testhelper"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"
	"go.viam.com/utils/pexec"
)

const (
	// NumPointClouds is the number of pointclouds saved in artifact
	// for the cartographer integration tests.
	NumPointClouds = 15
)

// IntegrationLidarReleasePointCloudChan is the lidar pointcloud release
// channel for the cartographer integration tests.
var IntegrationLidarReleasePointCloudChan = make(chan int, 1)

// Service in the internal package includes additional exported functions relating to the data and
// slam processes in the slam service. These functions are not exported to the user. This resolves
// a circular import caused by the inject package.
type Service interface {
	StartDataProcess(cancelCtx context.Context, lidar lidar.Lidar, c chan int)
	StartSLAMProcess(ctx context.Context) error
	StopSLAMProcess() error
	Close() error
	GetSLAMProcessConfig() pexec.ProcessConfig
	GetSLAMProcessBufferedLogReader() bufio.Reader
}

// SetupDeps returns the dependencies based on the sensors passed as arguments.
func SetupDeps(sensors []string) registry.Dependencies {
	deps := make(registry.Dependencies)

	for _, sensor := range sensors {
		switch sensor {
		case "good_lidar":
			deps[camera.Named(sensor)] = getGoodLidar()
		case "invalid_sensor":
			deps[camera.Named(sensor)] = getInvalidSensor()
		case "gibberish":
			return deps
		case "cartographer_int_lidar":
			deps[camera.Named(sensor)] = getIntegrationLidar()
		default:
			continue
		}
	}
	return deps
}

func getGoodLidar() *inject.Camera {
	cam := &inject.Camera{}
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
	return cam
}

func getInvalidSensor() *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return nil, errors.New("camera not lidar")
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	return cam
}

func getIntegrationLidar() *inject.Camera {
	cam := &inject.Camera{}
	var index uint64
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		select {
		case <-IntegrationLidarReleasePointCloudChan:
			i := atomic.AddUint64(&index, 1) - 1
			if i >= NumPointClouds {
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
	return cam
}

// ClearDirectory deletes the contents in the path directory
// without deleting path itself.
func ClearDirectory(t *testing.T, path string) {
	t.Helper()

	err := slamTesthelper.ResetFolder(path)
	test.That(t, err, test.ShouldBeNil)
}
