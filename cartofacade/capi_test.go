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

		cfg := GetBadTestConfig()
		algoCfg := GetTestAlgoConfig()
		vc, err := NewCarto(cfg, algoCfg, &pvcl)

		// initialize viam_carto incorrectly
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_DATA_DIR_NOT_PROVIDED"))
		test.That(t, vc, test.ShouldNotBeNil)

		cfg, dir, err := GetTestConfig("mysensor")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		algoCfg = GetTestAlgoConfig()
		vc, err = NewCarto(cfg, algoCfg, &pvcl)

		// initialize viam_carto correctly
		test.That(t, err, test.ShouldBeNil)
		test.That(t, vc, test.ShouldNotBeNil)

		// test start
		err = vc.Start()
		test.That(t, err, test.ShouldBeNil)

		// test invalid addSensorReading: not in sensor list
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.Local)
		err = vc.AddSensorReading("not my sensor", []byte("he0llo"), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_SENSOR_NOT_IN_SENSOR_LIST")

		// test invalid addSensorReading: empty reading
		err = vc.AddSensorReading("mysensor", []byte(""), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_SENSOR_READING_EMPTY")

		// test invalid addSensorReading: invalid reading
		err = vc.AddSensorReading("mysensor", []byte("he0llo"), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_SENSOR_READING_INVALID")

		// read PCD
		file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/0.pcd"))
		test.That(t, err, test.ShouldBeNil)
		buf := new(bytes.Buffer)
		pc, err := pointcloud.ReadPCD(file)
		test.That(t, err, test.ShouldBeNil)

		// test invalid addSensorReading: valid reading binary
		err = pointcloud.ToPCD(pc, buf, 1)
		test.That(t, err, test.ShouldBeNil)
		err = vc.AddSensorReading("mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeNil)

		// test invalid addSensorReading: valid reading ascii
		err = pointcloud.ToPCD(pc, buf, 0)
		test.That(t, err, test.ShouldBeNil)
		err = vc.AddSensorReading("mysensor", buf.Bytes(), timestamp)
		test.That(t, err, test.ShouldBeNil)

		// confirm the pointcloud package still doesn't support binary compressed
		// pointclouds. If it does, we need to implement:
		// https://viam.atlassian.net/browse/RSDK-3753
		err = pointcloud.ToPCD(pc, buf, 2)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "compressed PCD not yet implemented")

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
