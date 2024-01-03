package sensors

import (
	"context"
	"math"
	"time"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/camera/replaypcd"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/components/movementsensor/replay"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/utils/contextutils"
)

// BadTime can be used to represent something that should cause an error while parsing it as a time.
const BadTime = "NOT A TIME"

var (
	// TestTimestamp can be used to test specific timestamps provided by a replay sensor.
	TestTimestamp = time.Now().UTC().Format("2006-01-02T15:04:05.999999Z")
	// TestLinAcc is the successful mock linear acceleration result used for testing.
	TestLinAcc = r3.Vector{X: 1, Y: 2, Z: 3}
	// TestAngVel is the successful mock angular velocity result used for testing.
	TestAngVel = spatialmath.AngularVelocity{X: 1.1, Y: .5, Z: 0}
	// TestPosition is the successful mock position result used for testing.
	TestPosition = geo.NewPoint(5, 4)
	// TestOrientation is the successful mock orientation result used for testing.
	TestOrientation = &spatialmath.Quaternion{Real: 0.1, Imag: -0.2, Jmag: 2.5, Kmag: -9.1}
)

// TestSensor represents sensors used for testing.
type TestSensor string

const (
	// InvalidSensorTestErrMsg represents an error message that indicates that the sensor is invalid.
	InvalidSensorTestErrMsg = "invalid test sensor"

	// ---------- LIDAR Test Sensors --------------.

	// GoodLidar is a lidar that works as expected and returns a pointcloud.
	GoodLidar TestSensor = "good_lidar"
	// WarmingUpLidar is a lidar whose NextPointCloud function returns a "warming up" error.
	WarmingUpLidar TestSensor = "warming_up_lidar"
	// LidarWithErroringFunctions is a lidar whose functions return errors.
	LidarWithErroringFunctions TestSensor = "lidar_with_erroring_functions"
	// LidarWithInvalidProperties is a lidar whose properties are invalid.
	LidarWithInvalidProperties TestSensor = "lidar_with_invalid_properties"
	// GibberishLidar is a lidar that can't be found in the dependencies.
	GibberishLidar TestSensor = "gibberish_lidar"
	// NoLidar is a lidar that represents that no lidar is set up or added.
	NoLidar TestSensor = ""

	// ReplayLidar is a lidar that works as expected and returns a pointcloud.
	ReplayLidar TestSensor = "replay_lidar"
	// InvalidReplayLidar is a lidar whose meta timestamp is invalid.
	InvalidReplayLidar TestSensor = "invalid_replay_lidar"
	// FinishedReplayLidar is a lidar whose NextPointCloud function returns an end of dataset error.
	FinishedReplayLidar TestSensor = "finished_replay_lidar"

	// ------------- IMU Test Sensors ---------------.

	// GoodIMU is an IMU that works as expected and returns linear acceleration and angular velocity values.
	GoodIMU TestSensor = "good_imu"
	// IMUWithErroringFunctions is an IMU whose functions return errors.
	IMUWithErroringFunctions TestSensor = "imu_with_erroring_functions"

	// ReplayIMU is an IMU that works as expected and returns linear acceleration and angular velocity values.
	ReplayIMU TestSensor = "replay_imu"
	// InvalidReplayIMU is an IMU whose meta timestamp is invalid.
	InvalidReplayIMU TestSensor = "invalid_replay_imu"
	// FinishedReplayIMU is an IMU whose LinearAcceleration and AngularVelocity functions return an end of
	// dataset error.
	FinishedReplayIMU TestSensor = "finished_replay_imu"

	// -------------- ODOMETER Test Sensors ------------.

	// GoodOdometer is an odometer that works as expected and returns position and orientation values.
	GoodOdometer TestSensor = "good_odometer"
	// OdometerWithErroringFunctions is an Odometer whose functions return errors.
	OdometerWithErroringFunctions TestSensor = "odometer_with_erroring_functions"

	// ReplayOdometer is an odometer that works as expected and returns position and orientation values.
	ReplayOdometer TestSensor = "replay_odometer"
	// InvalidReplayOdometer is an odometer whose meta timestamp is invalid.
	InvalidReplayOdometer TestSensor = "invalid_replay_odometer"
	// FinishedReplayOdometer is an odometer whose Position and Orientation functions return an end of
	// dataset error.
	FinishedReplayOdometer TestSensor = "finished_replay_odometer"

	// ------------- IMU + ODOMETER Test Sensors ----------.

	// MovementSensorNotIMUNotOdometer is a movement sensor that does neither support an IMU nor an odometer.
	MovementSensorNotIMUNotOdometer TestSensor = "movement_sensor_not_imu_not_odometer"
	// GoodMovementSensorBothIMUAndOdometer is a movement sensor that supports both an IMU nor an odometer.
	GoodMovementSensorBothIMUAndOdometer TestSensor = "good_movement_sensor_imu_and_odometer"
	// MovementSensorBothIMUAndOdometerWithErroringFunctions is a movement sensor whose functions return errors.
	MovementSensorBothIMUAndOdometerWithErroringFunctions TestSensor = "movement_sensor_imu_and_odometer_with_erroring_functions"
	// MovementSensorWithErroringPropertiesFunc is a movement sensor whose Properties function returns an error.
	MovementSensorWithErroringPropertiesFunc TestSensor = "movement_sensor_with_erroring_properties_function"
	// MovementSensorWithInvalidProperties is a movement sensor whose properties are invalid.
	MovementSensorWithInvalidProperties TestSensor = "movement_sensor_with_invalid_properties"
	// GibberishMovementSensor is a movement sensor that can't be found in the dependencies.
	GibberishMovementSensor TestSensor = "gibberish_movement_sensor"
	// NoMovementSensor is a movement sensor that represents that no movement sensor is set up or added.
	NoMovementSensor TestSensor = ""

	// ReplayMovementSensorBothIMUAndOdometer is a replay movement sensor that supports both an IMU nor an odometer.
	ReplayMovementSensorBothIMUAndOdometer TestSensor = "replay_movement_sensor_imu_and_odometer"
	// InvalidReplayMovementSensorBothIMUAndOdometer is a replay movement sensor that supports both an IMU nor an odometer.
	InvalidReplayMovementSensorBothIMUAndOdometer TestSensor = "invalid_replay_movement_sensor_imu_and_odometer"
	// FinishedReplayMovementSensor is a movement sensor whose LinearAcceleration, AngularVelocity, Position, and Orientation
	//  functions return an end of dataset error.
	FinishedReplayMovementSensor TestSensor = "finished_replay_movement_sensor"
)

var (
	testLidars = map[TestSensor]func() *inject.Camera{
		GoodLidar:                  getGoodLidar,
		WarmingUpLidar:             getWarmingUpLidar,
		LidarWithErroringFunctions: getLidarWithErroringFunctions,
		LidarWithInvalidProperties: getLidarWithInvalidProperties,
		ReplayLidar:                func() *inject.Camera { return getReplayLidar(TestTimestamp) },
		InvalidReplayLidar:         func() *inject.Camera { return getReplayLidar(BadTime) },
		FinishedReplayLidar:        getFinishedReplayLidar,
	}

	testMovementSensors = map[TestSensor]func() *inject.MovementSensor{
		GoodIMU:                              getGoodIMU,
		IMUWithErroringFunctions:             getIMUWithErroringFunctions,
		ReplayIMU:                            func() *inject.MovementSensor { return getReplayIMU(TestTimestamp) },
		InvalidReplayIMU:                     func() *inject.MovementSensor { return getReplayIMU(BadTime) },
		FinishedReplayIMU:                    func() *inject.MovementSensor { return getFinishedReplayIMU() },
		GoodOdometer:                         getGoodOdometer,
		OdometerWithErroringFunctions:        getOdometerWithErroringFunctions,
		ReplayOdometer:                       func() *inject.MovementSensor { return getReplayOdometer(TestTimestamp) },
		InvalidReplayOdometer:                func() *inject.MovementSensor { return getReplayOdometer(BadTime) },
		FinishedReplayOdometer:               func() *inject.MovementSensor { return getFinishedReplayOdometer() },
		MovementSensorNotIMUNotOdometer:      getMovementSensorNotIMUAndNotOdometer,
		GoodMovementSensorBothIMUAndOdometer: getGoodMovementSensorBothIMUAndOdometer,
		MovementSensorBothIMUAndOdometerWithErroringFunctions: getMovementSensorBothIMUAndOdometerWithErroringFunctions,
		MovementSensorWithErroringPropertiesFunc:              getMovementSensorWithErroringPropertiesFunc,
		MovementSensorWithInvalidProperties:                   getMovementSensorWithInvalidProperties,
		ReplayMovementSensorBothIMUAndOdometer:                func() *inject.MovementSensor { return getReplayMovementSensor(TestTimestamp) },
		InvalidReplayMovementSensorBothIMUAndOdometer:         func() *inject.MovementSensor { return getReplayMovementSensor(BadTime) },
		FinishedReplayMovementSensor:                          func() *inject.MovementSensor { return getFinishedReplayMovementSensor() },
	}
)

// SetupDeps returns the dependencies based on the lidar and movement sensor names passed as arguments.
func SetupDeps(lidarName, movementSensorName TestSensor) resource.Dependencies {
	deps := make(resource.Dependencies)
	if getLidarFunc, ok := testLidars[lidarName]; ok {
		deps[camera.Named(string(lidarName))] = getLidarFunc()
	}

	if getMovementSensorFunc, ok := testMovementSensors[movementSensorName]; ok {
		deps[movementsensor.Named(string(movementSensorName))] = getMovementSensorFunc()
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
		return camera.Properties{SupportsPCD: true}, nil
	}
	return cam
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

func getLidarWithErroringFunctions() *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return nil, errors.New(InvalidSensorTestErrMsg)
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New(InvalidSensorTestErrMsg)
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
		return TestLinAcc, nil
	}
	imu.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return TestAngVel, nil
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
		return r3.Vector{}, errors.New(InvalidSensorTestErrMsg)
	}
	imu.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return spatialmath.AngularVelocity{}, errors.New(InvalidSensorTestErrMsg)
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
		return TestLinAcc, nil
	}
	imu.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return TestAngVel, nil
	}
	imu.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
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

func getGoodOdometer() *inject.MovementSensor {
	odometer := &inject.MovementSensor{}
	odometer.PositionFunc = func(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
		return TestPosition, 0.0, nil
	}
	odometer.OrientationFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
		return TestOrientation, nil
	}
	odometer.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			PositionSupported:    true,
			OrientationSupported: true,
		}, nil
	}
	return odometer
}

func getOdometerWithErroringFunctions() *inject.MovementSensor {
	odometer := &inject.MovementSensor{}
	odometer.PositionFunc = func(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
		return geo.NewPoint(math.NaN(), math.NaN()), math.NaN(), errors.New(InvalidSensorTestErrMsg)
	}
	odometer.OrientationFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
		return nil, errors.New(InvalidSensorTestErrMsg)
	}
	odometer.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			PositionSupported:    true,
			OrientationSupported: true,
		}, nil
	}
	return odometer
}

func getReplayOdometer(testTime string) *inject.MovementSensor {
	odometer := &inject.MovementSensor{}
	odometer.PositionFunc = func(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return TestPosition, 0.0, nil
	}
	odometer.OrientationFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return TestOrientation, nil
	}
	odometer.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			PositionSupported:    true,
			OrientationSupported: true,
		}, nil
	}
	return odometer
}

func getFinishedReplayOdometer() *inject.MovementSensor {
	odometer := &inject.MovementSensor{}
	odometer.PositionFunc = func(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
		return geo.NewPoint(math.NaN(), math.NaN()), math.NaN(), replay.ErrEndOfDataset
	}
	odometer.OrientationFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
		return nil, replay.ErrEndOfDataset
	}
	odometer.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			PositionSupported:    true,
			OrientationSupported: true,
		}, nil
	}
	return odometer
}

func getMovementSensorNotIMUAndNotOdometer() *inject.MovementSensor {
	movementSensor := &inject.MovementSensor{}
	movementSensor.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{}, nil
	}
	return movementSensor
}

func getGoodMovementSensorBothIMUAndOdometer() *inject.MovementSensor {
	movementSensor := &inject.MovementSensor{}
	movementSensor.PositionFunc = func(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
		return TestPosition, 0.0, nil
	}
	movementSensor.OrientationFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
		return TestOrientation, nil
	}
	movementSensor.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		return TestLinAcc, nil
	}
	movementSensor.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return TestAngVel, nil
	}
	movementSensor.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			OrientationSupported:        true,
			PositionSupported:           true,
			LinearAccelerationSupported: true,
		}, nil
	}
	return movementSensor
}

func getMovementSensorBothIMUAndOdometerWithErroringFunctions() *inject.MovementSensor {
	movementSensor := &inject.MovementSensor{}
	movementSensor.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		return r3.Vector{}, errors.New(InvalidSensorTestErrMsg)
	}
	movementSensor.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return spatialmath.AngularVelocity{}, errors.New(InvalidSensorTestErrMsg)
	}
	movementSensor.PositionFunc = func(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
		return geo.NewPoint(math.NaN(), math.NaN()), math.NaN(), errors.New(InvalidSensorTestErrMsg)
	}
	movementSensor.OrientationFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
		return nil, errors.New(InvalidSensorTestErrMsg)
	}
	movementSensor.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
			PositionSupported:           true,
			OrientationSupported:        true,
		}, nil
	}
	return movementSensor
}

func getMovementSensorWithErroringPropertiesFunc() *inject.MovementSensor {
	movementSensor := &inject.MovementSensor{}
	movementSensor.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{}, errors.New("error getting properties")
	}
	return movementSensor
}

func getMovementSensorWithInvalidProperties() *inject.MovementSensor {
	movementSensor := &inject.MovementSensor{}
	movementSensor.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		return TestLinAcc, nil
	}
	movementSensor.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return TestAngVel, nil
	}
	movementSensor.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    false,
			LinearAccelerationSupported: true,
			PositionSupported:           false,
			OrientationSupported:        true,
		}, nil
	}
	return movementSensor
}

func getReplayMovementSensor(testTime string) *inject.MovementSensor {
	movementSensor := &inject.MovementSensor{}
	movementSensor.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return TestLinAcc, nil
	}
	movementSensor.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return TestAngVel, nil
	}
	movementSensor.PositionFunc = func(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return TestPosition, 0.0, nil
	}
	movementSensor.OrientationFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return TestOrientation, nil
	}
	movementSensor.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
			PositionSupported:           true,
			OrientationSupported:        true,
		}, nil
	}
	return movementSensor
}

func getFinishedReplayMovementSensor() *inject.MovementSensor {
	movementSensor := &inject.MovementSensor{}
	movementSensor.LinearAccelerationFunc = func(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
		return r3.Vector{}, replay.ErrEndOfDataset
	}
	movementSensor.AngularVelocityFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
		return spatialmath.AngularVelocity{}, replay.ErrEndOfDataset
	}
	movementSensor.PositionFunc = func(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
		return geo.NewPoint(math.NaN(), math.NaN()), math.NaN(), replay.ErrEndOfDataset
	}
	movementSensor.OrientationFunc = func(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
		return nil, replay.ErrEndOfDataset
	}
	movementSensor.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{
			AngularVelocitySupported:    true,
			LinearAccelerationSupported: true,
			PositionSupported:           true,
			OrientationSupported:        true,
		}, nil
	}
	return movementSensor
}
