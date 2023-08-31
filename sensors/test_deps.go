package sensors

import (
	"context"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"github.com/viamrobotics/gostream"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/camera/replaypcd"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/components/movementsensor/replay"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/utils/contextutils"
)

const (
	// TestTime can be used to test specific timestamps provided by a replay sensor.
	TestTime = "2006-01-02T15:04:05.9999Z"
	// BadTime can be used to represent something that should cause an error while parsing it as a time.
	BadTime = "NOT A TIME"
)

var (
	// LinAcc is the successful mock linear acceleration result used for testing.
	LinAcc = r3.Vector{X: 1, Y: 1, Z: 1}
	// AngVel is the successful mock angular velocity  result used for testing.
	AngVel = spatialmath.AngularVelocity{X: 1, Y: .5, Z: 0}
)

// SetupDeps returns the dependencies based on the lidar passed as argument.
func SetupDeps(lidarName, imuName string) resource.Dependencies {
	deps := make(resource.Dependencies)
	switch lidarName {
	case "good_lidar":
		deps[camera.Named(lidarName)] = getGoodLidar()
	case "warming_up_lidar":
		deps[camera.Named(lidarName)] = getWarmingUpLidar()
	case "replay_lidar":
		deps[camera.Named(lidarName)] = getReplayLidar(TestTime)
	case "invalid_replay_lidar":
		deps[camera.Named(lidarName)] = getReplayLidar(BadTime)
	case "lidar_with_erroring_functions":
		deps[camera.Named(lidarName)] = getLidarWithErroringFunctions()
	case "lidar_with_invalid_properties":
		deps[camera.Named(lidarName)] = getLidarWithInvalidProperties()
	case "gibberish_lidar":
		return deps
	case "finished_replay_lidar":
		deps[camera.Named(lidarName)] = getFinishedReplayLidar()
	}

	// TODO: create setup deps for various replay_imu, see https://viam.atlassian.net/browse/RSDK-4556
	switch imuName {
	case "good_imu":
		deps[movementsensor.Named(imuName)] = getGoodIMU()
	case "replay_imu":
		deps[movementsensor.Named(imuName)] = getReplayIMU(TestTime)
	case "invalid_replay_imu":
		deps[movementsensor.Named(imuName)] = getReplayIMU(BadTime)
	case "imu_with_erroring_functions":
		deps[movementsensor.Named(imuName)] = getIMUWithErroringFunctions()
	case "imu_with_invalid_properties":
		deps[movementsensor.Named(imuName)] = getIMUWithInvalidProperties()
	case "gibberish_imu":
		return deps
	case "finished_replay_imu":
		deps[movementsensor.Named(imuName)] = getFinishedReplayIMU()
	}

	return deps
}

func getWarmingUpLidar() *inject.Camera {
	cam := &inject.Camera{}
	counter := 0
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		counter++
		if counter == 1 {
			return nil, errors.Errorf("warming up %d", counter)
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
		return camera.Properties{SupportsPCD: true}, nil
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
		return camera.Properties{SupportsPCD: true}, nil
	}
	return cam
}

func getReplayLidar(testTime string) *inject.Camera {
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
		return camera.Properties{SupportsPCD: true}, nil
	}
	return cam
}

func getLidarWithInvalidProperties() *inject.Camera {
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
		return camera.Properties{SupportsPCD: false}, nil
	}
	return cam
}

func getLidarWithErroringFunctions() *inject.Camera {
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
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{SupportsPCD: true}, nil
	}
	return cam
}

func getFinishedReplayLidar() *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return nil, replaypcd.ErrEndOfDataset
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{SupportsPCD: true}, nil
	}
	return cam
}

func getGoodIMU() *inject.MovementSensor {
	imu := &inject.MovementSensor{}
	imu.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		return LinAcc, nil
	}
	imu.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return AngVel, nil
	}
	imu.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
		}, nil
	}
	return imu
}

func getReplayIMU(testTime string) *inject.MovementSensor {
	imu := &inject.MovementSensor{}
	imu.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return LinAcc, nil
	}
	imu.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return AngVel, nil
	}
	imu.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
		}, nil
	}
	return imu
}

func getIMUWithErroringFunctions() *inject.MovementSensor {
	imu := &inject.MovementSensor{}
	imu.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		return r3.Vector{}, errors.New("invalid sensor")
	}
	imu.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return spatialmath.AngularVelocity{}, errors.New("invalid sensor")
	}
	imu.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
		}, nil
	}
	return imu
}

func getIMUWithInvalidProperties() *inject.MovementSensor {
	imu := &inject.MovementSensor{}
	imu.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		return LinAcc, nil
	}
	imu.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return AngVel, nil
	}
	imu.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    false,
			LinearAccelerationSupported: false,
		}, nil
	}
	return imu
}

func getFinishedReplayIMU() *inject.MovementSensor {
	imu := &inject.MovementSensor{}
	imu.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		return r3.Vector{}, replay.ErrEndOfDataset
	}
	imu.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return spatialmath.AngularVelocity{}, replay.ErrEndOfDataset
	}
	imu.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
		}, nil
	}
	return imu
}
