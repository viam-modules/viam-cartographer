package cartofacade

import (
	"errors"
	"testing"
	"time"

	"go.viam.com/test"
)

func getTestConfig() CartoConfig {
	return CartoConfig{
		Sensors:            []string{"rplidar", "imu"},
		MapRateSecond:      5,
		DataDir:            "temp",
		ComponentReference: "component",
		LidarConfig:        twoD,
	}
}

func getTestAlgoConfig() CartoAlgoConfig {
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
	t.Run("config properly converted between C and go", func(t *testing.T) {
		cfg := getTestConfig()
		vcc, err := getConfig(cfg)
		test.That(t, err, test.ShouldBeNil)

		sensors := bStringToGoStringSlice(vcc.sensors, int(vcc.sensors_len))
		test.That(t, sensors[0], test.ShouldResemble, "rplidar")
		test.That(t, sensors[1], test.ShouldResemble, "imu")
		test.That(t, vcc.sensors_len, test.ShouldEqual, 2)

		dataDir := bstringToGoString(vcc.data_dir)
		test.That(t, dataDir, test.ShouldResemble, "temp")

		componentReference := bstringToGoString(vcc.component_reference)
		test.That(t, componentReference, test.ShouldResemble, "component")
		freeBstringArray(vcc.sensors, vcc.sensors_len)

		test.That(t, vcc.lidar_config, test.ShouldEqual, twoD)
	})
}

func TestGetPositionResponse(t *testing.T) {
	gpr := getTestGetPositionResponse()
	t.Run("position response properly converted between C and go", func(t *testing.T) {
		holder := toGetPositionResponse(gpr)
		test.That(t, holder.ComponentReference, test.ShouldEqual, "C++ component reference")

		test.That(t, holder.X, test.ShouldAlmostEqual, 100, .001)
		test.That(t, holder.Y, test.ShouldAlmostEqual, 200, .001)
		test.That(t, holder.Z, test.ShouldAlmostEqual, 300, .001)

		test.That(t, holder.Ox, test.ShouldAlmostEqual, 400, .001)
		test.That(t, holder.Oy, test.ShouldAlmostEqual, 500, .001)
		test.That(t, holder.Oz, test.ShouldAlmostEqual, 600, .001)

		test.That(t, holder.Imag, test.ShouldAlmostEqual, 700, .001)
		test.That(t, holder.Jmag, test.ShouldAlmostEqual, 800, .001)
		test.That(t, holder.Kmag, test.ShouldAlmostEqual, 900, .001)

		test.That(t, holder.Theta, test.ShouldAlmostEqual, 1000, .001)
		test.That(t, holder.Real, test.ShouldAlmostEqual, 1100, .001)
	})
}

func TestToSensorReading(t *testing.T) {
	t.Run("sensor reading properly converted between c and go", func(t *testing.T) {
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.Local)
		sr := toSensorReading([]byte("he0llo"), timestamp)
		test.That(t, bstringToGoString(sr.sensor_reading), test.ShouldResemble, "he0llo")
		test.That(t, sr.sensor_reading_time_unix_micro, test.ShouldEqual, timestamp.UnixMicro())
	})
}

func TestBstringToByteSlice(t *testing.T) {
	t.Run("b strings are properly converted to byte slices", func(t *testing.T) {
		bstring := goStringToBstring("hell0!")
		bytes := bstringToByteSlice(bstring)
		test.That(t, bytes, test.ShouldResemble, []byte("hell0!"))
	})
}

func TestCGoAPI(t *testing.T) {
	pvcl, err := NewLib(1, 1)

	t.Run("initialize viam_carto_lib", func(t *testing.T) {
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pvcl, test.ShouldNotBeNil)
	})

	cfg := getTestConfig()
	algoCfg := getTestAlgoConfig()
	vc, err := New(cfg, algoCfg, pvcl)
	t.Run("initialize viam_carto", func(t *testing.T) {
		test.That(t, err, test.ShouldBeNil)
		test.That(t, vc, test.ShouldNotBeNil)
	})

	t.Run("test start", func(t *testing.T) {
		err = vc.Start()
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("test addSensorReading", func(t *testing.T) {
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.Local)
		err := vc.AddSensorReading([]byte("he0llo"), timestamp)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("test getPosition", func(t *testing.T) {
		holder, err := vc.GetPosition()

		test.That(t, err, test.ShouldBeNil)
		test.That(t, holder.ComponentReference, test.ShouldEqual, "C++ component reference")

		test.That(t, holder.X, test.ShouldAlmostEqual, 100, .001)
		test.That(t, holder.Y, test.ShouldAlmostEqual, 200, .001)
		test.That(t, holder.Z, test.ShouldAlmostEqual, 300, .001)

		test.That(t, holder.Ox, test.ShouldAlmostEqual, 400, .001)
		test.That(t, holder.Oy, test.ShouldAlmostEqual, 500, .001)
		test.That(t, holder.Oz, test.ShouldAlmostEqual, 600, .001)

		test.That(t, holder.Imag, test.ShouldAlmostEqual, 700, .001)
		test.That(t, holder.Jmag, test.ShouldAlmostEqual, 800, .001)
		test.That(t, holder.Kmag, test.ShouldAlmostEqual, 900, .001)

		test.That(t, holder.Theta, test.ShouldAlmostEqual, 1000, .001)
		test.That(t, holder.Real, test.ShouldAlmostEqual, 1100, .001)
	})

	t.Run("test getPointCloudMap", func(t *testing.T) {
		_, err = vc.GetPointCloudMap()
		test.That(t, err, test.ShouldResemble, errors.New("nil pointcloud"))
	})

	t.Run("test getInternalState", func(t *testing.T) {
		_, err = vc.GetInternalState()
		test.That(t, err, test.ShouldResemble, errors.New("nil internal state"))
	})

	t.Run("test stop", func(t *testing.T) {
		err = vc.Stop()
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("terminate viam_carto", func(t *testing.T) {
		err = vc.Terminate()
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("terminate viam_carto_lib", func(t *testing.T) {
		err = pvcl.Terminate()
		test.That(t, err, test.ShouldBeNil)
	})
}
