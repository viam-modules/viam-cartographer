// Package testhelper provides test helpers which don't depend on viamcartographer
package testhelper

import (
	"context"
	"os"
	"strconv"
	"sync/atomic"

	"github.com/pkg/errors"
	"github.com/viamrobotics/gostream"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/utils/contextutils"
	"go.viam.com/utils/artifact"
)

const (
	// NumPointClouds is the number of pointclouds saved in artifact
	// for the cartographer integration tests.
	NumPointClouds = 15
	// TestTime can be used to test specific timestamps provided by a replay sensor.
	TestTime = "2006-01-02T15:04:05.9999Z"
	// BadTime can be used to represent something that should cause an error while parsing it as a time.
	BadTime = "NOT A TIME"
)

// IntegrationLidarReleasePointCloudChan is the lidar pointcloud release
// channel for the integration tests.
var IntegrationLidarReleasePointCloudChan = make(chan int, 1)

// SetupDeps returns the dependencies based on the sensors passed as arguments.
func SetupDeps(sensors []string) resource.Dependencies {
	deps := make(resource.Dependencies)

	for _, sensor := range sensors {
		switch sensor {
		case "good_lidar":
			deps[camera.Named(sensor)] = getGoodLidar()
		case "warming_up_lidar":
			deps[camera.Named(sensor)] = getWarmingUpLidar()
		case "replay_sensor":
			deps[camera.Named(sensor)] = getReplaySensor(TestTime)
		case "invalid_replay_sensor":
			deps[camera.Named(sensor)] = getReplaySensor(BadTime)
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

func getWarmingUpLidar() *inject.Camera {
	cam := &inject.Camera{}
	couter := 0
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		counter++
		if counter == 1 {
			return nil, errors.Errorf("warming up %d", couter)
		}
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

func getReplaySensor(testTime string) *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
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
		return nil, errors.New("invalid sensor")
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("invalid sensor")
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
			file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/" + strconv.FormatUint(i, 10) + ".pcd"))
			if err != nil {
				return nil, err
			}
			pointCloud, err := pointcloud.ReadPCD(file)
			if err != nil {
				return nil, err
			}
			return pointCloud, nil
		default:
			return nil, errors.Errorf("Lidar not ready to return point cloud %v", atomic.LoadUint64(&index))
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
