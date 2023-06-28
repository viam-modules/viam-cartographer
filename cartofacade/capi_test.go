package cartofacade

import (
	"bytes"
	"errors"
	"os"
	"testing"
	"time"

	"go.viam.com/rdk/pointcloud"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"
)

func TestGetConfig(t *testing.T) {
	t.Run("config properly converted between C and go", func(t *testing.T) {
		cfg, dir, err := GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		vcc, err := getConfig(cfg)
		test.That(t, err, test.ShouldBeNil)

		sensors := bStringToGoStringSlice(vcc.sensors, int(vcc.sensors_len))
		test.That(t, sensors[0], test.ShouldResemble, "mysensor")
		test.That(t, sensors[1], test.ShouldResemble, "imu")
		test.That(t, vcc.sensors_len, test.ShouldEqual, 2)

		dataDir := bstringToGoString(vcc.data_dir)
		test.That(t, dataDir, test.ShouldResemble, dir)

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
		test.That(t, holder.Imag, test.ShouldEqual, 700)
		test.That(t, holder.Jmag, test.ShouldEqual, 800)
		test.That(t, holder.Kmag, test.ShouldEqual, 900)
		test.That(t, holder.Real, test.ShouldEqual, 1100)
	})
}

func TestToSensorReading(t *testing.T) {
	t.Run("sensor reading properly converted between c and go", func(t *testing.T) {
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		sr := toSensorReading("mysensor", []byte("he0llo"), timestamp)
		test.That(t, bstringToGoString(sr.sensor), test.ShouldResemble, "mysensor")
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
	pvcl, err := NewLib(0, 1)

	t.Run("test state machine", func(t *testing.T) {
		// initialize viam_carto_lib
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pvcl, test.ShouldNotBeNil)

		cfg, dir, err := GetTestConfig("mysensor")
		defer os.RemoveAll(dir)

		test.That(t, err, test.ShouldBeNil)

		algoCfg := GetTestAlgoConfig()
		vc, err := NewCarto(cfg, algoCfg, &CartoLibMock{})

		// initialize viam_carto with an invalid library incorrectly
		test.That(t, err, test.ShouldResemble, errors.New("cannot cast provided library to a CartoLib"))
		test.That(t, vc, test.ShouldNotBeNil)

		cfgBad := GetBadTestConfig()
		vc, err = NewCarto(cfgBad, algoCfg, &pvcl)
		// initialize viam_carto incorrectly
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_DATA_DIR_NOT_PROVIDED"))
		test.That(t, vc, test.ShouldNotBeNil)

		algoCfg = GetTestAlgoConfig()
		vc, err = NewCarto(cfg, algoCfg, &pvcl)

		// initialize viam_carto correctly
		test.That(t, err, test.ShouldBeNil)
		test.That(t, vc, test.ShouldNotBeNil)

		// test start
		err = vc.start()
		test.That(t, err, test.ShouldBeNil)

		// test invalid addSensorReading: not in sensor list
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		err = vc.addSensorReading("not my sensor", []byte("he0llo"), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_SENSOR_NOT_IN_SENSOR_LIST")

		// test invalid addSensorReading: empty reading
		err = vc.addSensorReading("mysensor", []byte(""), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_SENSOR_READING_EMPTY")

		// test invalid addSensorReading: invalid reading
		err = vc.addSensorReading("mysensor", []byte("he0llo"), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_SENSOR_READING_INVALID")

		// read PCD
		file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/0.pcd"))
		test.That(t, err, test.ShouldBeNil)
		buf := new(bytes.Buffer)
		pc, err := pointcloud.ReadPCD(file)
		test.That(t, err, test.ShouldBeNil)

		// test valid addSensorReading: valid reading binary
		err = pointcloud.ToPCD(pc, buf, 1)
		test.That(t, err, test.ShouldBeNil)
		err = vc.addSensorReading("mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeNil)

		// test valid addSensorReading: valid reading ascii
		err = pointcloud.ToPCD(pc, buf, 0)
		test.That(t, err, test.ShouldBeNil)
		err = vc.addSensorReading("mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeNil)

		// confirm the pointcloud package still doesn't support binary compressed
		// pointclouds. If it does, we need to implement:
		// https://viam.atlassian.net/browse/RSDK-3753
		err = pointcloud.ToPCD(pc, buf, 2)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "compressed PCD not yet implemented")

		// test getPosition
		holder, err := vc.getPosition()

		test.That(t, err, test.ShouldBeNil)
		test.That(t, holder.ComponentReference, test.ShouldEqual, "mysensor")

		test.That(t, holder.X, test.ShouldEqual, 0)
		test.That(t, holder.Y, test.ShouldEqual, 0)
		test.That(t, holder.Z, test.ShouldEqual, 0)

		test.That(t, holder.Imag, test.ShouldEqual, 0)
		test.That(t, holder.Jmag, test.ShouldEqual, 0)
		test.That(t, holder.Kmag, test.ShouldEqual, 0)
		test.That(t, holder.Real, test.ShouldEqual, 1)

		// test getPointCloudMap
		_, err = vc.getPointCloudMap()
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_POINTCLOUD_MAP_EMPTY"))

		// test getInternalState
		_, err = vc.getInternalState()
		test.That(t, err, test.ShouldResemble, errors.New("nil internal state"))

		// test stop
		err = vc.stop()
		test.That(t, err, test.ShouldBeNil)

		// terminate viam_carto
		err = vc.terminate()
		test.That(t, err, test.ShouldBeNil)

		// terminate viam_carto_lib
		err = pvcl.Terminate()
		test.That(t, err, test.ShouldBeNil)
	})
}
