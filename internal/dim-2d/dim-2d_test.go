// Package dim2d implements the 2D sub algorithm
package dim2d

import (
	"context"
	"os"
	"testing"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	slamConfig "go.viam.com/slam/config"
	"go.viam.com/slam/sensors/lidar"
	slamTesthelper "go.viam.com/slam/testhelper"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
)

func TestNewLidar(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("No sensor provided", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: "data",
		}
		deps := testhelper.SetupDeps(attrCfg.Sensors)
		actualLidar, err := NewLidar(context.Background(), deps, attrCfg, logger)
		expectedLidar := lidar.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("New cartographer slam service with more than one sensor", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"lidar", "one-too-many"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: "data",
		}

		deps := testhelper.SetupDeps(attrCfg.Sensors)
		actualLidar, err := NewLidar(context.Background(), deps, attrCfg, logger)
		expectedLidar := lidar.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: 'sensors' must contain only one lidar camera, but is 'sensors: [lidar, one-too-many]'"))
	})

	t.Run("New cartographer slam service with non-existing sensor", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"gibberish"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: "data",
		}

		deps := testhelper.SetupDeps(attrCfg.Sensors)
		actualLidar, err := NewLidar(context.Background(), deps, attrCfg, logger)
		expectedLidar := lidar.Lidar{}
		test.That(t, actualLidar, test.ShouldResemble, expectedLidar)
		test.That(t, err, test.ShouldBeError,
			errors.New("configuring lidar camera error: error getting lidar camera "+
				"gibberish for slam service: \"gibberish\" missing from dependencies"))
	})

	t.Run("New cartographer slam service with good lidar in slam mode 2d", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: "data",
		}
		ctx := context.Background()
		deps := testhelper.SetupDeps(attrCfg.Sensors)
		actualLidar, err := NewLidar(ctx, deps, attrCfg, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, attrCfg.Sensors[0])
		test.That(t, err, test.ShouldBeNil)

		pc, err := actualLidar.GetData(ctx)
		test.That(t, pc, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeNil)
	})
}

func TestGetAndSaveData(t *testing.T) {
	ctx := context.Background()
	logger := golog.NewTestLogger(t)
	dataDir, err := slamTesthelper.CreateTempFolderArchitecture(logger)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Test with good lidar in slam mode 2d", func(t *testing.T) {
		attrCfg := &slamConfig.AttrConfig{
			Sensors:       []string{"good_lidar"},
			ConfigParams:  map[string]string{"mode": "2d"},
			DataDirectory: "data",
		}
		deps := testhelper.SetupDeps(attrCfg.Sensors)
		actualLidar, err := NewLidar(ctx, deps, attrCfg, logger)
		test.That(t, actualLidar.Name, test.ShouldEqual, attrCfg.Sensors[0])
		test.That(t, err, test.ShouldBeNil)

		GetAndSaveData(ctx, dataDir, actualLidar, logger)

		files, err := os.ReadDir(dataDir + "/data/")
		test.That(t, len(files), test.ShouldEqual, 1)
		test.That(t, err, test.ShouldBeNil)
	})

	// Will uncomment & update or delete this test once
	// this PR is merged: https://github.com/viamrobotics/slam/pull/182
	// t.Run("Test with invalid lidar", func(t *testing.T) {
	// 	invalidLidar := lidar.Lidar{}
	// 	test.That(t, invalidLidar.Name, test.ShouldEqual, "")

	// 	GetAndSaveData(ctx, dataDir, invalidLidar, logger)

	// 	files, err := os.ReadDir(dataDir + "/data/")
	// 	test.That(t, len(files), test.ShouldEqual, 0)
	// 	test.That(t, err, test.ShouldEqual, errors.New("lidar is nil, can not get a pointcloud"))
	// })

	testhelper.ClearDirectory(t, dataDir)
}
