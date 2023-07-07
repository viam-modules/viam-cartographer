package testhelper

import (
	"context"
	"errors"

	"github.com/viamrobotics/gostream"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/utils/contextutils"
)

const (
	// TestTime can be used to test specific timestamps provided by a replay sensor.
	TestTime = "2006-01-02T15:04:05.9999Z"
	// BadTime can be used to represent something that should cause an error while parsing it as a time.
	BadTime = "NOT A TIME"
)

func SetupDeps(sensors []string) resource.Dependencies {
	deps := make(resource.Dependencies)

	for _, sensor := range sensors {
		switch sensor {
		case "good_lidar":
			deps[camera.Named(sensor)] = getGoodLidar()
		case "replay_sensor":
			deps[camera.Named(sensor)] = getReplaySensor(TestTime)
		case "invalid_replay_sensor":
			deps[camera.Named(sensor)] = getReplaySensor(BadTime)
		case "invalid_sensor":
			deps[camera.Named(sensor)] = getInvalidSensor()
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
