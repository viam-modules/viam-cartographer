package cartofacade

import (
	"errors"
	"os"
	"testing"
	"time"

	"go.viam.com/test"
)

func getTestConfig() (CartoConfig, string, error) {
	dir, err := os.MkdirTemp("", "slam-test")
	if err != nil {
		return CartoConfig{}, "", err
	}

	return CartoConfig{
		Sensors:            []string{"rplidar", "imu"},
		MapRateSecond:      5,
		DataDir:            dir,
		ComponentReference: "component",
		LidarConfig:        twoD,
	}, dir, nil
}

func getBadTestConfig() CartoConfig {
	return CartoConfig{
		Sensors:     []string{"rplidar", "imu"},
		LidarConfig: twoD,
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
		cfg, dir, err := getTestConfig()
		test.That(t, err, test.ShouldBeNil)
		defer os.RemoveAll(dir)

		vcc, err := getConfig(cfg)
		test.That(t, err, test.ShouldBeNil)

		sensors := bStringToGoStringSlice(vcc.sensors, int(vcc.sensors_len))
		test.That(t, sensors[0], test.ShouldResemble, "rplidar")
		test.That(t, sensors[1], test.ShouldResemble, "imu")
		test.That(t, vcc.sensors_len, test.ShouldEqual, 2)

		dataDir := bstringToGoString(vcc.data_dir)
		test.That(t, dataDir, test.ShouldResemble, dir)

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

		test.That(t, holder.X, test.ShouldEqual, 100)
		test.That(t, holder.Y, test.ShouldEqual, 200)
		test.That(t, holder.Z, test.ShouldEqual, 300)

		test.That(t, holder.Ox, test.ShouldEqual, 400)
		test.That(t, holder.Oy, test.ShouldEqual, 500)
		test.That(t, holder.Oz, test.ShouldEqual, 600)

		test.That(t, holder.Imag, test.ShouldEqual, 700)
		test.That(t, holder.Jmag, test.ShouldEqual, 800)
		test.That(t, holder.Kmag, test.ShouldEqual, 900)

		test.That(t, holder.Theta, test.ShouldEqual, 1000)
		test.That(t, holder.Real, test.ShouldEqual, 1100)
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

	t.Run("test state machine", func(t *testing.T) {
		// initialize viam_carto_lib
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pvcl, test.ShouldNotBeNil)

		cfg := getBadTestConfig()
		algoCfg := getTestAlgoConfig()
		vc, err := New(cfg, algoCfg, pvcl)

		// initialize viam_carto incorrectly
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_DATA_DIR_NOT_PROVIDED"))
		test.That(t, vc, test.ShouldNotBeNil)

		cfg, dir, err := getTestConfig()
		test.That(t, err, test.ShouldBeNil)
		defer os.RemoveAll(dir)

		algoCfg = getTestAlgoConfig()
		vc, err = New(cfg, algoCfg, pvcl)

		// initialize viam_carto correctly
		test.That(t, err, test.ShouldBeNil)
		test.That(t, vc, test.ShouldNotBeNil)

		// test start
		err = vc.Start()
		test.That(t, err, test.ShouldBeNil)

		// test addSensorReading
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.Local)
		err = vc.AddSensorReading([]byte("he0llo"), timestamp)
		test.That(t, err, test.ShouldBeNil)

		// test getPosition
		holder, err := vc.GetPosition()

		test.That(t, err, test.ShouldBeNil)
		test.That(t, holder.ComponentReference, test.ShouldEqual, "C++ component reference")

		test.That(t, holder.X, test.ShouldEqual, 100)
		test.That(t, holder.X, test.ShouldEqual, 100)
		test.That(t, holder.Y, test.ShouldEqual, 200)
		test.That(t, holder.Z, test.ShouldEqual, 300)

		test.That(t, holder.Ox, test.ShouldEqual, 400)
		test.That(t, holder.Oy, test.ShouldEqual, 500)
		test.That(t, holder.Oz, test.ShouldEqual, 600)

		test.That(t, holder.Imag, test.ShouldEqual, 700)
		test.That(t, holder.Jmag, test.ShouldEqual, 800)
		test.That(t, holder.Kmag, test.ShouldEqual, 900)

		test.That(t, holder.Theta, test.ShouldEqual, 1000)
		test.That(t, holder.Real, test.ShouldEqual, 1100)

		// test getPointCloudMap
		_, err = vc.GetPointCloudMap()
		test.That(t, err, test.ShouldResemble, errors.New("nil pointcloud"))

		// test getInternalState
		_, err = vc.GetInternalState()
		test.That(t, err, test.ShouldResemble, errors.New("nil internal state"))

		// test stop
		err = vc.Stop()
		test.That(t, err, test.ShouldBeNil)

		// terminate viam_carto
		err = vc.Terminate()
		test.That(t, err, test.ShouldBeNil)

		// terminate viam_carto_lib
		err = pvcl.Terminate()
		test.That(t, err, test.ShouldBeNil)
	})
}
