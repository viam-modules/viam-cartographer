package cartofacade

import (
	"context"
	"testing"
	"time"

	"go.viam.com/test"
)

func getTestConfig() CartoConfig {
	return CartoConfig{
		sensors:            []string{"rplidar", "imu"},
		mapRateSecond:      5,
		dataDir:            "data-dir",
		componentReference: "component",
		lidarConfig:        twoD,
	}
}

func getAlgoTestConfig() CartoAlgoConfig {
	return CartoAlgoConfig{
		optimizeOnStart:      false,
		optimizeEveryNNodes:  0,
		numRangeData:         0,
		missingDataRayLength: 0.0,
		maxRange:             0.0,
		minRange:             0.0,
		maxSubmapsToKeep:     0,
		freshSubmapsCount:    0,
		minCoveredArea:       0.0,
		minAddedSubmapsCount: 0,
		occupiedSpaceWeight:  0.0,
		translationWeight:    0.0,
		rotationWeight:       0.0,
	}
}

func TestGetConfig(t *testing.T) {
	t.Run("config properly converte between c and go", func(t *testing.T) {
		cfg := getTestConfig()
		vcc := getConfig(cfg)

		sensors := bStringToGoStringSlice(vcc.sensors, int(vcc.sensors_len))
		test.That(t, sensors[0], test.ShouldResemble, "rplidar")
		test.That(t, sensors[1], test.ShouldResemble, "imu")
		test.That(t, vcc.sensors_len, test.ShouldEqual, 2)

		dataDir := bstringToGoString(vcc.data_dir)
		test.That(t, dataDir, test.ShouldResemble, "data-dir")

		componentReference := bstringToGoString(vcc.component_reference)
		test.That(t, componentReference, test.ShouldResemble, "component")
		FreeBstringArray(vcc.sensors)

		test.That(t, vcc.lidar_config, test.ShouldEqual, twoD)
	})
}

func TestCGoAPI(t *testing.T) {
	pvcl, err := NewCartoLib(1, 1)

	t.Run("initialize viam_carto_lib", func(t *testing.T) {
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pvcl, test.ShouldNotBeNil)
	})

	cfg := getTestConfig()
	algoCfg := getAlgoTestConfig()
	vc, err := NewCarto(cfg, algoCfg, pvcl)
	t.Run("initialize viam_carto", func(t *testing.T) {
		test.That(t, err, test.ShouldBeNil)
		test.That(t, vc, test.ShouldNotBeNil)
	})

	t.Run("test start", func(t *testing.T) {
		err = vc.Start(context.Background())
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("test addSensorReading", func(t *testing.T) {
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.Local)
		err = vc.AddSensorReading(context.Background(), []byte("he0llo"), timestamp)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("test getPosition", func(t *testing.T) {
		holder, err := vc.GetPosition(context.Background())

		test.That(t, err, test.ShouldBeNil)
		test.That(t, holder.compReference, test.ShouldEqual, "C++ component reference")

		test.That(t, holder.x, test.ShouldAlmostEqual, 100, .001)
		test.That(t, holder.y, test.ShouldAlmostEqual, 200, .001)
		test.That(t, holder.z, test.ShouldAlmostEqual, 300, .001)

		test.That(t, holder.ox, test.ShouldAlmostEqual, 400, .001)
		test.That(t, holder.oy, test.ShouldAlmostEqual, 500, .001)
		test.That(t, holder.oz, test.ShouldAlmostEqual, 600, .001)

		test.That(t, holder.imag, test.ShouldAlmostEqual, 700, .001)
		test.That(t, holder.jmag, test.ShouldAlmostEqual, 800, .001)
		test.That(t, holder.kmag, test.ShouldAlmostEqual, 900, .001)

		test.That(t, holder.theta, test.ShouldAlmostEqual, 1000, .001)
		test.That(t, holder.real, test.ShouldAlmostEqual, 1100, .001)
	})

	t.Run("test getPointCloudMap", func(t *testing.T) {
		_, err = vc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("test getInternalState", func(t *testing.T) {
		_, err = vc.GetInternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("test stop", func(t *testing.T) {
		err = vc.Stop(context.Background())
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("terminate viam_carto", func(t *testing.T) {
		err = vc.TerminateCarto(context.Background())
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("terminate viam_carto_lib", func(t *testing.T) {
		err = pvcl.TerminateCartoLib()
		test.That(t, err, test.ShouldBeNil)
	})
}
