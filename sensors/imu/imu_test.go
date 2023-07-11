package imu_test

import (
	"context"
	"image/color"
	"testing"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/sensors/imu"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
)

const (
	testIMUName  = "testIMUName"
	wrongIMUName = "wrongIMUName"
)

func TestNew(t *testing.T) {
	t.Run("Empty sensors array failure", func(t *testing.T) {
		injectedIMU := &inject.MovementSensor{}
		deps := make(resource.Dependencies)
		deps[movementsensor.Named(testIMUName)] = injectedIMU
		sensors := []string{}
		sensorIndex := 0

		testIMU, err := imu.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldEqual, "index out of bounds")
		test.That(t, testIMU, test.ShouldResemble, imu.IMU{})
	})

	t.Run("Negative sensor index failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{testIMUName}
		sensorIndex := -1

		testIMU, err := imu.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldEqual, "index out of bounds")
		test.That(t, testIMU, test.ShouldResemble, imu.IMU{})
	})

	t.Run("Sensor index out of sensor array bounds failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{testIMUName}
		sensorIndex := 1

		testIMU, err := imu.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldEqual, "index out of bounds")
		test.That(t, testIMU, test.ShouldResemble, imu.IMU{})
	})

	t.Run("Empty dependencies empty sensors failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{}
		sensorIndex := 0

		testIMU, err := imu.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldEqual, "index out of bounds")
		test.That(t, testIMU, test.ShouldResemble, imu.IMU{})
	})

	t.Run("Empty dependencies non-empty sensors failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{testIMUName}
		sensorIndex := 0
		expectedErrorMessage := "error getting lidar camera " + testIMUName + " for slam service"

		testIMU, err := imu.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, expectedErrorMessage)
		test.That(t, testIMU, test.ShouldResemble, imu.IMU{})
	})

	t.Run("Wrong sensor name failure", func(t *testing.T) {
		deps := make(resource.Dependencies)
		sensors := []string{wrongIMUName}
		sensorIndex := 0
		expectedErrorMessage := "error getting lidar camera " + wrongIMUName + " for slam service"

		testIMU, err := imu.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, expectedErrorMessage)
		test.That(t, testIMU, test.ShouldResemble, imu.IMU{})
	})

	t.Run("Successful creation of IMU", func(t *testing.T) {
		injectedIMU := &inject.MovementSensor{}
		deps := make(resource.Dependencies)
		deps[camera.Named(testIMUName)] = injectedIMU
		sensors := []string{testIMUName}
		sensorIndex := 0

		testIMU, err := imu.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, testIMU, test.ShouldNotBeNil)
		test.That(t, testIMU.Name, test.ShouldEqual, testIMUName)
	})
}

func TestGetAngularVelocity(t *testing.T) {
	t.Run("Successful GetAngularVelocity call", func(t *testing.T) {
		injectedIMU := &inject.MovementSensor{}
		cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
			pc1 := pointcloud.New()
			err := pc1.Set(pointcloud.NewVector(1, 0, 0), pointcloud.NewColoredData(color.NRGBA{255, 0, 0, 255}))
			test.That(t, err, test.ShouldBeNil)
			return pc1, nil
		}
		deps := make(resource.Dependencies)
		deps[camera.Named(testLidarName)] = cam
		sensors := []string{testLidarName}
		sensorIndex := 0

		testLidar, err := lidar.New(deps, sensors, sensorIndex)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, testLidar, test.ShouldNotBeNil)
		test.That(t, testLidar.Name, test.ShouldEqual, testLidarName)

		pc, err := testLidar.GetData(context.Background())
		test.That(t, pc, test.ShouldNotBeNil)
		test.That(t, pc.Size(), test.ShouldEqual, 1)
		test.That(t, err, test.ShouldBeNil)

		data, got := pc.At(1, 0, 0)
		test.That(t, got, test.ShouldBeTrue)
		test.That(t, data.Color(), test.ShouldResemble, &color.NRGBA{255, 0, 0, 255})
	})
}
