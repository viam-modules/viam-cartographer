// Package dim2d_test implements tests for the 2D sub algorithm
package dim2d_test

import (
	"context"
	"os"
	"testing"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
	vcTesthelper "github.com/viamrobotics/viam-cartographer/testhelper"
	"go.viam.com/test"

	dim2d "github.com/viamrobotics/viam-cartographer/internal/dim-2d"
	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
)

func TestNewLidar(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("No sensor provided", func(t *testing.T) {
		sensors := []string{}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(context.Background(), deps, sensors, logger)
		expectedLidar := lidar.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("Failed lidar creation due to more than one sensor provided", func(t *testing.T) {
		sensors := []string{"lidar", "one-too-many"}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(context.Background(), deps, sensors, logger)
		expectedLidar := lidar.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: 'sensors' must contain only one lidar camera, but is 'sensors: [lidar, one-too-many]'"))
	})

	t.Run("Failed lidar creation with non-existing sensor", func(t *testing.T) {
		sensors := []string{"gibberish"}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(context.Background(), deps, sensors, logger)
		expectedLidar := lidar.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: error getting lidar camera "+
				"gibberish for slam service: \"rdk:component:camera/gibberish\" missing from dependencies"))
	})

	t.Run("Successful lidar creation", func(t *testing.T) {
		sensors := []string{"good_lidar"}
		ctx := context.Background()
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(ctx, deps, sensors, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, sensors[0])
		test.That(t, err, test.ShouldBeNil)

		pc, err := actualLidar.GetData(ctx)
		test.That(t, pc, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeNil)
	})
}

func TestGetAndSaveData(t *testing.T) {
	ctx := context.Background()
	logger := golog.NewTestLogger(t)
	dataDir, err := vcTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Successful call to GetAndSaveData", func(t *testing.T) {
		sensors := []string{"good_lidar"}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(ctx, deps, sensors, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, sensors[0])
		test.That(t, err, test.ShouldBeNil)

		_, err = dim2d.GetAndSaveData(ctx, dataDir, actualLidar, logger)
		test.That(t, err, test.ShouldBeNil)

		files, err := os.ReadDir(dataDir + "/data/")
		test.That(t, len(files), test.ShouldEqual, 1)
		test.That(t, err, test.ShouldBeNil)
	})

	testhelper.ClearDirectory(t, dataDir)
}

func TestValidateGetAndSaveData(t *testing.T) {
	ctx := context.Background()
	logger := golog.NewTestLogger(t)
	dataDir, err := vcTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Successful call to ValidateGetAndSaveData", func(t *testing.T) {
		sensors := []string{"good_lidar"}
		deps := testhelper.SetupDeps(sensors)
		actualLidar, err := dim2d.NewLidar(ctx, deps, sensors, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, sensors[0])
		test.That(t, err, test.ShouldBeNil)

		err = dim2d.ValidateGetAndSaveData(ctx,
			dataDir,
			actualLidar,
			testhelper.SensorValidationMaxTimeoutSecForTest,
			testhelper.SensorValidationMaxTimeoutSecForTest,
			logger,
		)
		test.That(t, err, test.ShouldBeNil)

		files, err := os.ReadDir(dataDir + "/data/")
		test.That(t, len(files), test.ShouldEqual, 0)
		test.That(t, err, test.ShouldBeNil)
	})

	testhelper.ClearDirectory(t, dataDir)
}
