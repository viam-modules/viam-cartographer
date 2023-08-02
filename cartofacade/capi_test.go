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

func positionIsZero(t *testing.T, position GetPosition) {
	test.That(t, position.X, test.ShouldEqual, 0)
	test.That(t, position.Y, test.ShouldEqual, 0)
	test.That(t, position.Z, test.ShouldEqual, 0)

	test.That(t, position.Imag, test.ShouldEqual, 0)
	test.That(t, position.Jmag, test.ShouldEqual, 0)
	test.That(t, position.Kmag, test.ShouldEqual, 0)
	test.That(t, position.Real, test.ShouldEqual, 1)
}

// confirm the pointcloud package still doesn't support binary compressed
// pointclouds. If it does, we need to implement:
// https://viam.atlassian.net/browse/RSDK-3753
func confirmBinaryCompressedUnsupported(t *testing.T) {
	file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/0.pcd"))
	test.That(t, err, test.ShouldBeNil)
	pc, err := pointcloud.ReadPCD(file)
	test.That(t, err, test.ShouldBeNil)

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(pc, buf, pointcloud.PCDCompressed)
	test.That(t, err, test.ShouldBeError)
	test.That(t, err.Error(), test.ShouldResemble, "compressed PCD not yet implemented")
}

func testAddSensorReading(t *testing.T, vc Carto, pcdPath string, timestamp time.Time, pcdType pointcloud.PCDType) {
	file, err := os.Open(artifact.MustPath(pcdPath))
	test.That(t, err, test.ShouldBeNil)

	pc, err := pointcloud.ReadPCD(file)
	test.That(t, err, test.ShouldBeNil)

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(pc, buf, pcdType)
	test.That(t, err, test.ShouldBeNil)

	err = vc.addSensorReading("mysensor", buf.Bytes(), timestamp)
	test.That(t, err, test.ShouldBeNil)
}

func TestGetConfig(t *testing.T) {
	t.Run("config properly converted between C and go", func(t *testing.T) {
		cfg, dir, err := GetTestConfig("mysensor", "")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		vcc, err := getConfig(cfg)
		test.That(t, err, test.ShouldBeNil)

		camera := bstringToGoString(vcc.camera)
		test.That(t, camera, test.ShouldResemble, "mysensor")

		dataDir := bstringToGoString(vcc.data_dir)
		test.That(t, dataDir, test.ShouldResemble, dir)

		test.That(t, vcc.lidar_config, test.ShouldEqual, TwoD)
	})
}

func TestGetConfigWithMovementSensor(t *testing.T) {
	t.Run("config properly converted between C and go", func(t *testing.T) {
		cfg, dir, err := GetTestConfig("mylidar", "myIMU")
		defer os.RemoveAll(dir)
		test.That(t, err, test.ShouldBeNil)

		vcc, err := getConfig(cfg)
		test.That(t, err, test.ShouldBeNil)

		camera := bstringToGoString(vcc.camera)
		test.That(t, camera, test.ShouldResemble, "mylidar")

		movementSensor := bstringToGoString(vcc.movement_sensor)
		test.That(t, movementSensor, test.ShouldResemble, "myIMU")

		dataDir := bstringToGoString(vcc.data_dir)
		test.That(t, dataDir, test.ShouldResemble, dir)

		test.That(t, vcc.lidar_config, test.ShouldEqual, TwoD)
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
		test.That(t, sr.sensor_reading_time_unix_milli, test.ShouldEqual, timestamp.UnixMilli())
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

		cfg, dir, err := GetTestConfig("mysensor", "")
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
		test.That(t, vc.SlamMode, test.ShouldEqual, MappingMode)

		// test start
		err = vc.start()
		test.That(t, err, test.ShouldBeNil)

		// test getPosition before sensor data is added
		position, err := vc.getPosition()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position.ComponentReference, test.ShouldEqual, "mysensor")
		positionIsZero(t, position)

		// test getPointCloudMap before sensor data is added
		pcd, err := vc.getPointCloudMap()
		test.That(t, pcd, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_POINTCLOUD_MAP_EMPTY"))

		// test getInternalState before sensor data is added
		internalState, err := vc.getInternalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldBeGreaterThan, 0)
		lastInternalState := internalState

		// test invalid addSensorReading: not in sensor list
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		err = vc.addSensorReading("not my sensor", []byte("he0llo"), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_SENSOR_NOT_IN_SENSOR_LIST")

		// test invalid addSensorReading: empty reading
		timestamp = timestamp.Add(time.Second * 2)
		err = vc.addSensorReading("mysensor", []byte(""), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_SENSOR_READING_EMPTY")

		// test invalid addSensorReading: invalid reading
		timestamp = timestamp.Add(time.Second * 2)
		err = vc.addSensorReading("mysensor", []byte("he0llo"), timestamp)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_SENSOR_READING_INVALID")

		confirmBinaryCompressedUnsupported(t)

		// NOTE: This test is very carefully created in order to not hit
		// cases where cartographer won't update the map for whatever reason.
		// For example, if you change the time increments from 2 seconds to 1
		// second, the map doesn't update (at least not after 10 lidar readings).
		// It is not clear why cartographer has this behavior.
		// Cartographer does not provide any feedback regarding
		// why or when it does / does not update the map.
		// As a result, these tests show best case behavior.

		// 1. test valid addSensorReading: valid reading ascii
		t.Log("sensor reading 1")
		timestamp = timestamp.Add(time.Second * 2)
		testAddSensorReading(t, vc, "viam-cartographer/mock_lidar/0.pcd", timestamp, pointcloud.PCDAscii)

		// test getPosition zeroed if not enough sensor data has been provided
		position, err = vc.getPosition()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position.ComponentReference, test.ShouldEqual, "mysensor")
		positionIsZero(t, position)

		// test getPointCloudMap returns error if not enough sensor data has been provided
		pcd, err = vc.getPointCloudMap()
		test.That(t, pcd, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_POINTCLOUD_MAP_EMPTY"))

		// test getInternalState always returns non empty results
		internalState, err = vc.getInternalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldBeGreaterThan, 0)
		test.That(t, internalState, test.ShouldNotEqual, lastInternalState)
		lastInternalState = internalState

		// 2. test valid addSensorReading: valid reading binary
		t.Log("sensor reading 2")
		timestamp = timestamp.Add(time.Second * 2)
		testAddSensorReading(t, vc, "viam-cartographer/mock_lidar/1.pcd", timestamp, pointcloud.PCDBinary)

		// test getPosition zeroed
		position, err = vc.getPosition()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position.ComponentReference, test.ShouldEqual, "mysensor")
		positionIsZero(t, position)

		// test getPointCloudMap now returns a non empty result
		pcd, err = vc.getPointCloudMap()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcd, test.ShouldNotBeNil)
		pc, err := pointcloud.ReadPCD(bytes.NewReader(pcd))
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc.Size(), test.ShouldNotEqual, 0)

		// test getInternalState always returns different non empty results than first call
		internalState, err = vc.getInternalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldBeGreaterThan, 0)
		test.That(t, internalState, test.ShouldNotEqual, lastInternalState)
		lastInternalState = internalState

		// third sensor reading populates the pointcloud map and the position
		t.Log("sensor reading 3")
		timestamp = timestamp.Add(time.Second * 2)
		testAddSensorReading(t, vc, "viam-cartographer/mock_lidar/2.pcd", timestamp, pointcloud.PCDBinary)

		// test getPosition, is no longer zeroed
		position, err = vc.getPosition()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position.ComponentReference, test.ShouldEqual, "mysensor")
		test.That(t, position.X, test.ShouldNotEqual, 0)
		test.That(t, position.Y, test.ShouldNotEqual, 0)
		test.That(t, position.Z, test.ShouldEqual, 0)
		test.That(t, position.Imag, test.ShouldEqual, 0)
		test.That(t, position.Jmag, test.ShouldEqual, 0)
		test.That(t, position.Kmag, test.ShouldNotEqual, 0)
		test.That(t, position.Real, test.ShouldNotEqual, 1)

		// test getPointCloudMap returns non 0 response
		// on arm64 linux this returns a different response
		// than the last call to getPointCloudMap()
		// on arm64 osx it returns the same map as
		// the last call to getPointCloudMap()
		// https://viam.atlassian.net/browse/RSDK-3866
		pcd, err = vc.getPointCloudMap()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcd, test.ShouldNotBeNil)
		pc, err = pointcloud.ReadPCD(bytes.NewReader(pcd))
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc.Size(), test.ShouldNotEqual, 0)

		// test getInternalState always returns different non empty results than second call
		internalState, err = vc.getInternalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldBeGreaterThan, 0)
		test.That(t, internalState, test.ShouldNotEqual, lastInternalState)

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
