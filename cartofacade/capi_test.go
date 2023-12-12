package cartofacade

import (
	"bytes"
	"errors"
	"os"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

func positionIsZero(t *testing.T, position Position) {
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

func testAddLidarReading(t *testing.T, vc Carto, pcdPath string, timestamp time.Time, pcdType pointcloud.PCDType) {
	file, err := os.Open(artifact.MustPath(pcdPath))
	test.That(t, err, test.ShouldBeNil)

	pc, err := pointcloud.ReadPCD(file)
	test.That(t, err, test.ShouldBeNil)

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(pc, buf, pcdType)
	test.That(t, err, test.ShouldBeNil)

	reading := s.TimedLidarReadingResponse{
		Reading:     buf.Bytes(),
		ReadingTime: timestamp,
	}

	err = vc.addLidarReading("my-lidar", reading)
	test.That(t, err, test.ShouldBeNil)
}

func TestGetConfig(t *testing.T) {
	t.Run("config properly converted between C and go with no movement sensor specified", func(t *testing.T) {
		cfg := GetTestConfig("my-lidar", "", "", true)
		vcc, err := getConfig(cfg)
		test.That(t, err, test.ShouldBeNil)

		camera := bstringToGoString(vcc.camera)
		test.That(t, camera, test.ShouldResemble, "my-lidar")

		enableMapping := bool(vcc.enable_mapping)
		test.That(t, enableMapping, test.ShouldBeTrue)

		test.That(t, vcc.lidar_config, test.ShouldEqual, TwoD)
	})

	t.Run("config properly converted between C and go with a movement sensor specified", func(t *testing.T) {
		cfg := GetTestConfig("my-lidar", "my-movement-sensor", "", true)
		vcc, err := getConfig(cfg)
		test.That(t, err, test.ShouldBeNil)

		camera := bstringToGoString(vcc.camera)
		test.That(t, camera, test.ShouldResemble, "my-lidar")

		movementSensor := bstringToGoString(vcc.movement_sensor)
		test.That(t, movementSensor, test.ShouldResemble, "my-movement-sensor")

		enableMapping := bool(vcc.enable_mapping)
		test.That(t, enableMapping, test.ShouldBeTrue)

		test.That(t, vcc.lidar_config, test.ShouldEqual, TwoD)
	})

	t.Run("config properly converted between C and go with a movement sensor specified and an existing map", func(t *testing.T) {
		filename := "viam-cartographer/outputs/viam-office-02-22-3/internal_state/internal_state_0.pbstream"

		cfg := GetTestConfig("my-lidar", "my-movement-sensor", filename, false)
		vcc, err := getConfig(cfg)
		test.That(t, err, test.ShouldBeNil)

		camera := bstringToGoString(vcc.camera)
		test.That(t, camera, test.ShouldResemble, "my-lidar")

		movementSensor := bstringToGoString(vcc.movement_sensor)
		test.That(t, movementSensor, test.ShouldResemble, "my-movement-sensor")

		enableMapping := bool(vcc.enable_mapping)
		test.That(t, enableMapping, test.ShouldBeFalse)

		test.That(t, vcc.lidar_config, test.ShouldEqual, TwoD)
	})
}

func TestPositionResponse(t *testing.T) {
	gpr := getTestPositionResponse()
	t.Run("position response properly converted between C and go", func(t *testing.T) {
		holder := toPositionResponse(gpr)
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

func TestToLidarReading(t *testing.T) {
	t.Run("lidar reading properly converted between c and go", func(t *testing.T) {
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		reading := s.TimedLidarReadingResponse{
			Reading:     []byte("he0llo"),
			ReadingTime: timestamp,
		}
		sr := toLidarReading("my-lidar", reading)
		test.That(t, bstringToGoString(sr.lidar), test.ShouldResemble, "my-lidar")
		test.That(t, bstringToGoString(sr.lidar_reading), test.ShouldResemble, "he0llo")
		test.That(t, sr.lidar_reading_time_unix_milli, test.ShouldEqual, timestamp.UnixMilli())
	})
}

func TestToIMUReading(t *testing.T) {
	t.Run("IMU reading properly converted between c and go", func(t *testing.T) {
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		reading := s.TimedIMUReadingResponse{
			LinearAcceleration: r3.Vector{X: 0.1, Y: 0, Z: 9.8},
			AngularVelocity:    spatialmath.AngularVelocity{X: 0, Y: -0.2, Z: 0},
			ReadingTime:        timestamp,
		}
		sr := toIMUReading("my-movement-sensor", reading)
		test.That(t, bstringToGoString(sr.imu), test.ShouldResemble, "my-movement-sensor")
		test.That(t, sr.lin_acc_x, test.ShouldEqual, reading.LinearAcceleration.X)
		test.That(t, sr.lin_acc_y, test.ShouldEqual, reading.LinearAcceleration.Y)
		test.That(t, sr.lin_acc_z, test.ShouldEqual, reading.LinearAcceleration.Z)
		test.That(t, sr.ang_vel_x, test.ShouldEqual, reading.AngularVelocity.X)
		test.That(t, sr.ang_vel_y, test.ShouldEqual, reading.AngularVelocity.Y)
		test.That(t, sr.ang_vel_z, test.ShouldEqual, reading.AngularVelocity.Z)
		test.That(t, sr.imu_reading_time_unix_milli, test.ShouldEqual, timestamp.UnixMilli())
	})
}

func TestToOdometerReading(t *testing.T) {
	t.Run("odometer reading properly converted between c and go", func(t *testing.T) {
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		reading := s.TimedOdometerReadingResponse{
			Position:    geo.NewPoint(4, 5),
			Orientation: &spatialmath.Quaternion{Real: 0.8, Imag: -0.2, Jmag: 5.6, Kmag: -0.7},
			ReadingTime: timestamp,
		}
		origin := geo.NewPoint(0, 0)
		translation := spatialmath.GeoPointToPoint(reading.Position, origin)
		sr := toOdometerReading("my-movement-sensor", reading)
		test.That(t, bstringToGoString(sr.odometer), test.ShouldResemble, "my-movement-sensor")
		test.That(t, sr.translation_x, test.ShouldEqual, translation.X)
		test.That(t, sr.translation_y, test.ShouldEqual, translation.Y)
		test.That(t, sr.translation_z, test.ShouldEqual, translation.Z)
		test.That(t, sr.rotation_x, test.ShouldEqual, reading.Orientation.Quaternion().Imag)
		test.That(t, sr.rotation_y, test.ShouldEqual, reading.Orientation.Quaternion().Jmag)
		test.That(t, sr.rotation_z, test.ShouldEqual, reading.Orientation.Quaternion().Kmag)
		test.That(t, sr.rotation_w, test.ShouldEqual, reading.Orientation.Quaternion().Real)
		test.That(t, sr.odometer_reading_time_unix_milli, test.ShouldEqual, timestamp.UnixMilli())
	})
}

func TestBstringToByteSlice(t *testing.T) {
	t.Run("b strings are properly converted to byte slices", func(t *testing.T) {
		bstring := goStringToBstring("hell0!")
		bytes := bstringToByteSlice(bstring)
		test.That(t, bytes, test.ShouldResemble, []byte("hell0!"))
	})
}

func TestCGoAPIWithoutMovementSensor(t *testing.T) {
	pvcl, err := NewLib(0, 1)

	t.Run("test state machine without movement sensor", func(t *testing.T) {
		// initialize viam_carto_lib
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pvcl, test.ShouldNotBeNil)

		cfg := GetTestConfig("my-lidar", "", "", true)
		algoCfg := GetTestAlgoConfig(false)
		vc, err := NewCarto(cfg, algoCfg, &CartoLibMock{})

		// initialize viam_carto with an invalid library incorrectly
		test.That(t, err, test.ShouldResemble, errors.New("cannot cast provided library to a CartoLib"))
		test.That(t, vc, test.ShouldNotBeNil)

		cfgBad := GetBadTestConfig()
		vc, err = NewCarto(cfgBad, algoCfg, &pvcl)
		// initialize viam_carto incorrectly
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_COMPONENT_REFERENCE_INVALID"))
		test.That(t, vc, test.ShouldNotBeNil)

		algoCfg = GetTestAlgoConfig(false)
		vc, err = NewCarto(cfg, algoCfg, &pvcl)

		// initialize viam_carto correctly
		test.That(t, err, test.ShouldBeNil)
		test.That(t, vc, test.ShouldNotBeNil)
		test.That(t, vc.SlamMode, test.ShouldEqual, MappingMode)

		// test start
		err = vc.start()
		test.That(t, err, test.ShouldBeNil)

		// test position before sensor data is added
		position, err := vc.position()
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_GET_POSITION_NOT_INITIALIZED"))
		test.That(t, position, test.ShouldResemble, Position{})

		// test pointCloudMap before sensor data is added
		pcd, err := vc.pointCloudMap()
		test.That(t, pcd, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_POINTCLOUD_MAP_EMPTY"))

		// test internalState before sensor data is added
		internalState, err := vc.internalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldBeGreaterThan, 0)
		lastInternalState := internalState

		// test invalid addLidarReading: sensor name unknown
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		reading := s.TimedLidarReadingResponse{
			Reading:     []byte("he0llo"),
			ReadingTime: timestamp,
		}
		err = vc.addLidarReading("not my sensor", reading)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_UNKNOWN_SENSOR_NAME")

		// test invalid addLidarReading: empty reading
		timestamp = timestamp.Add(time.Second * 2)
		reading = s.TimedLidarReadingResponse{
			Reading:     []byte(""),
			ReadingTime: timestamp,
		}
		err = vc.addLidarReading("my-lidar", reading)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_LIDAR_READING_EMPTY")

		// test invalid addLidarReading: invalid reading
		timestamp = timestamp.Add(time.Second * 2)
		reading = s.TimedLidarReadingResponse{
			Reading:     []byte("he0llo"),
			ReadingTime: timestamp,
		}
		err = vc.addLidarReading("my-lidar", reading)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_LIDAR_READING_INVALID")

		// test position should be unchanged by failed attempt to add data
		position, err = vc.position()
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_GET_POSITION_NOT_INITIALIZED"))
		test.That(t, position, test.ShouldResemble, Position{})

		// test pointCloudMap should be unchanged by failed attempt to add data
		pcd, err = vc.pointCloudMap()
		test.That(t, pcd, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_POINTCLOUD_MAP_EMPTY"))

		// test internalState should be unchanged by failed attempt to add data
		internalState, err = vc.internalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldEqual, len(lastInternalState))
		lastInternalState = internalState

		confirmBinaryCompressedUnsupported(t)

		// NOTE: This test is very carefully created in order to not hit
		// cases where cartographer won't update the map for whatever reason.
		// For example, if you change the time increments from 2 seconds to 1
		// second, the map doesn't update (at least not after 10 lidar readings).
		// It is not clear why cartographer has this behavior.
		// Cartographer does not provide any feedback regarding
		// why or when it does / does not update the map.
		// As a result, these tests show best case behavior.

		// 1. test valid addLidarReading: valid reading ascii
		t.Log("lidar reading 1")
		timestamp = timestamp.Add(time.Second * 2)
		testAddLidarReading(t, vc, "viam-cartographer/mock_lidar/0.pcd", timestamp, pointcloud.PCDAscii)

		// test position not initialized after first sensor data has been provided
		position, err = vc.position()
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_GET_POSITION_NOT_INITIALIZED"))
		test.That(t, position, test.ShouldResemble, Position{})

		// test pointCloudMap returns error if not enough sensor data has been provided
		pcd, err = vc.pointCloudMap()
		test.That(t, pcd, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_POINTCLOUD_MAP_EMPTY"))

		// test internalState always returns non empty results
		internalState, err = vc.internalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldBeGreaterThan, 0)
		test.That(t, internalState, test.ShouldNotEqual, lastInternalState)
		lastInternalState = internalState

		// 2. test valid addLidarReading: valid reading binary
		t.Log("lidar reading 2")
		timestamp = timestamp.Add(time.Second * 2)
		testAddLidarReading(t, vc, "viam-cartographer/mock_lidar/1.pcd", timestamp, pointcloud.PCDBinary)

		// test position zeroed
		position, err = vc.position()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position.ComponentReference, test.ShouldEqual, "my-lidar")
		positionIsZero(t, position)

		// test pointCloudMap now returns a non empty result
		pcd, err = vc.pointCloudMap()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcd, test.ShouldNotBeNil)
		pc, err := pointcloud.ReadPCD(bytes.NewReader(pcd))
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc.Size(), test.ShouldNotEqual, 0)

		// test internalState always returns different non empty results than first call
		internalState, err = vc.internalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldBeGreaterThan, 0)
		test.That(t, internalState, test.ShouldNotEqual, lastInternalState)
		lastInternalState = internalState

		// third sensor reading populates the pointcloud map and the position
		t.Log("lidar reading 3")
		timestamp = timestamp.Add(time.Second * 2)
		testAddLidarReading(t, vc, "viam-cartographer/mock_lidar/2.pcd", timestamp, pointcloud.PCDBinary)

		// test position, is no longer zeroed
		position, err = vc.position()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position.ComponentReference, test.ShouldEqual, "my-lidar")
		test.That(t, position.X, test.ShouldNotEqual, 0)
		test.That(t, position.Y, test.ShouldNotEqual, 0)
		test.That(t, position.Z, test.ShouldEqual, 0)
		test.That(t, position.Imag, test.ShouldEqual, 0)
		test.That(t, position.Jmag, test.ShouldEqual, 0)
		test.That(t, position.Kmag, test.ShouldNotEqual, 0)
		test.That(t, position.Real, test.ShouldNotEqual, 1)

		// test pointCloudMap returns non 0 response
		// on arm64 linux this returns a different response
		// than the last call to pointCloudMap()
		// on arm64 osx it returns the same map as
		// the last call to pointCloudMap()
		// https://viam.atlassian.net/browse/RSDK-3866
		pcd, err = vc.pointCloudMap()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcd, test.ShouldNotBeNil)
		pc, err = pointcloud.ReadPCD(bytes.NewReader(pcd))
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc.Size(), test.ShouldNotEqual, 0)

		// test internalState always returns different non empty results than second call
		internalState, err = vc.internalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldBeGreaterThan, 0)
		test.That(t, internalState, test.ShouldNotEqual, lastInternalState)

		// test runFinalOptimization succeeds
		err = vc.runFinalOptimization()
		test.That(t, err, test.ShouldBeNil)

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

func TestCGoAPIWithMovementSensor(t *testing.T) {
	pvcl, err := NewLib(0, 1)

	t.Run("test state machine with movement sensor", func(t *testing.T) {
		// initialize viam_carto_lib
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pvcl, test.ShouldNotBeNil)

		cfg := GetTestConfig("my-lidar", "", "", true)

		// test invalid IMU enabling configuration
		algoCfg := GetTestAlgoConfig(true)
		vc, err := NewCarto(cfg, algoCfg, &pvcl)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_IMU_PROVIDED_AND_IMU_ENABLED_MISMATCH"))
		test.That(t, vc, test.ShouldNotBeNil)

		cfg = GetTestConfig("my-lidar", "my-movement-sensor", "", true)
		algoCfg = GetTestAlgoConfig(true)
		vc, err = NewCarto(cfg, algoCfg, &pvcl)

		// initialize viam_carto correctly
		test.That(t, err, test.ShouldBeNil)
		test.That(t, vc, test.ShouldNotBeNil)
		test.That(t, vc.SlamMode, test.ShouldEqual, MappingMode)

		// test start
		err = vc.start()
		test.That(t, err, test.ShouldBeNil)

		// test position before sensor data is added
		position, err := vc.position()
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_GET_POSITION_NOT_INITIALIZED"))
		test.That(t, position, test.ShouldResemble, Position{})

		// test pointCloudMap before sensor data is added
		pcd, err := vc.pointCloudMap()
		test.That(t, pcd, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_POINTCLOUD_MAP_EMPTY"))

		// test internalState before sensor data is added
		internalState, err := vc.internalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldBeGreaterThan, 0)
		lastInternalState := internalState

		// test invalid addLidarReading: sensor name unknown
		timestamp := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		lidarReading := s.TimedLidarReadingResponse{
			Reading:     []byte("he0llo"),
			ReadingTime: timestamp,
		}
		err = vc.addLidarReading("not my sensor", lidarReading)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_UNKNOWN_SENSOR_NAME")

		// test invalid addIMUReading: sensor name unknown
		timestamp = time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		imuReading := s.TimedIMUReadingResponse{
			LinearAcceleration: r3.Vector{X: 0, Y: 0, Z: 9.8},
			AngularVelocity:    spatialmath.AngularVelocity{X: 0, Y: 0, Z: 0},
			ReadingTime:        timestamp,
		}
		err = vc.addIMUReading("not my sensor", imuReading)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_UNKNOWN_SENSOR_NAME")

		// test invalid addOdometerReading: sensor name unknown
		timestamp = time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)
		odometerReading := s.TimedOdometerReadingResponse{
			Position:    geo.NewPoint(4, 5),
			Orientation: &spatialmath.Quaternion{Real: 0.8, Imag: -0.2, Jmag: 5.6, Kmag: -0.7},
			ReadingTime: timestamp,
		}
		err = vc.addOdometerReading("not my sensor", odometerReading)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldResemble, "VIAM_CARTO_UNKNOWN_SENSOR_NAME")

		// test position should be unchanged by failed attempt to add data
		position, err = vc.position()
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_GET_POSITION_NOT_INITIALIZED"))
		test.That(t, position, test.ShouldResemble, Position{})

		// test pointCloudMap should be unchanged by failed attempt to add data
		pcd, err = vc.pointCloudMap()
		test.That(t, pcd, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError)
		test.That(t, err, test.ShouldResemble, errors.New("VIAM_CARTO_POINTCLOUD_MAP_EMPTY"))

		// test internalState should be unchanged by failed attempt to add data
		internalState, err = vc.internalState()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalState, test.ShouldNotBeNil)
		test.That(t, len(internalState), test.ShouldEqual, len(lastInternalState))
		lastInternalState = internalState

		confirmBinaryCompressedUnsupported(t)

		// NOTE: This test is very carefully created in order to not hit
		// cases where cartographer won't update the map for whatever reason.
		// It is not clear why cartographer has this behavior.
		// Cartographer does not provide any feedback regarding
		// why or when it does / does not update the map.
		// As a result, these tests show best case behavior.

		// 1. test valid addLidarReading: valid reading ascii
		tDelta := time.Second * 5
		movementSensorReadingOffset := time.Millisecond * 5
		t.Log("lidar reading 1")
		timestamp = timestamp.Add(tDelta)
		testAddLidarReading(t, vc, "viam-cartographer/mock_lidar/0.pcd", timestamp, pointcloud.PCDAscii)

		// test valid addIMUReading with closely followed timestamp
		t.Log("IMU reading 1")
		testIMUReading := s.TimedIMUReadingResponse{
			LinearAcceleration: r3.Vector{X: 0, Y: 0, Z: 9.8},
			AngularVelocity:    spatialmath.AngularVelocity{X: 0, Y: 0, Z: 0},
			ReadingTime:        timestamp.Add(movementSensorReadingOffset),
		}
		err = vc.addIMUReading("my-movement-sensor", testIMUReading)
		test.That(t, err, test.ShouldBeNil)

		// test valid addOdometerReading with closely followed timestamp
		t.Log("odometer reading 1")
		testOdometerReading := s.TimedOdometerReadingResponse{
			Position:    geo.NewPoint(4, 5),
			Orientation: &spatialmath.Quaternion{Real: 0.8, Imag: -0.2, Jmag: 5.6, Kmag: -0.7},
			ReadingTime: timestamp.Add(2 * movementSensorReadingOffset),
		}
		err = vc.addOdometerReading("my-movement-sensor", testOdometerReading)
		test.That(t, err, test.ShouldBeNil)

		// 2. test valid addLidarReading: valid reading binary
		t.Log("lidar reading 2")
		timestamp = timestamp.Add(tDelta)
		testAddLidarReading(t, vc, "viam-cartographer/mock_lidar/1.pcd", timestamp, pointcloud.PCDBinary)

		// test valid addIMUReading with closely followed timestamp
		t.Log("IMU reading 2")
		testIMUReading = s.TimedIMUReadingResponse{
			LinearAcceleration: r3.Vector{X: 0.1, Y: 0, Z: 9.8},
			AngularVelocity:    spatialmath.AngularVelocity{X: 0, Y: -0.2, Z: 0},
			ReadingTime:        timestamp.Add(movementSensorReadingOffset),
		}
		err = vc.addIMUReading("my-movement-sensor", testIMUReading)
		test.That(t, err, test.ShouldBeNil)

		// test valid addOdometerReading with closely followed timestamp
		t.Log("odometer reading 2")
		testOdometerReading = s.TimedOdometerReadingResponse{
			Position:    geo.NewPoint(5, 6),
			Orientation: &spatialmath.Quaternion{Real: 0.7, Imag: -0.1, Jmag: 5.4, Kmag: -0.8},
			ReadingTime: timestamp.Add(2 * movementSensorReadingOffset),
		}
		err = vc.addOdometerReading("my-movement-sensor", testOdometerReading)
		test.That(t, err, test.ShouldBeNil)

		// third sensor reading populates the pointcloud map and the position
		t.Log("lidar reading 3")
		timestamp = timestamp.Add(tDelta)
		testAddLidarReading(t, vc, "viam-cartographer/mock_lidar/2.pcd", timestamp, pointcloud.PCDBinary)

		// test valid addIMUReading with closely followed timestamp
		t.Log("IMU reading 3")
		testIMUReading = s.TimedIMUReadingResponse{
			LinearAcceleration: r3.Vector{X: 0.2, Y: 0, Z: 9.8},
			AngularVelocity:    spatialmath.AngularVelocity{X: -0.6, Y: 0, Z: 0},
			ReadingTime:        timestamp.Add(movementSensorReadingOffset),
		}
		err = vc.addIMUReading("my-movement-sensor", testIMUReading)
		test.That(t, err, test.ShouldBeNil)

		// test valid addOdometerReading with closely followed timestamp
		t.Log("odometer reading 3")
		testOdometerReading = s.TimedOdometerReadingResponse{
			Position:    geo.NewPoint(6, 7),
			Orientation: &spatialmath.Quaternion{Real: 0.4, Imag: 0.1, Jmag: 4.4, Kmag: -0.1},
			ReadingTime: timestamp.Add(2 * movementSensorReadingOffset),
		}
		err = vc.addOdometerReading("my-movement-sensor", testOdometerReading)
		test.That(t, err, test.ShouldBeNil)

		// test position is not zeroed
		position, err = vc.position()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, position.ComponentReference, test.ShouldEqual, "my-lidar")
		test.That(t, r3.Vector{X: position.X, Y: position.Y, Z: position.Z}, test.ShouldNotResemble, r3.Vector{X: 0, Y: 0, Z: 0})

		// test pointCloudMap returns a non empty result
		pcd, err = vc.pointCloudMap()
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pcd, test.ShouldNotBeNil)
		pc, err := pointcloud.ReadPCD(bytes.NewReader(pcd))
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc.Size(), test.ShouldNotEqual, 0)

		// test internalState always returns different non empty results than first call
		internalState, err = vc.internalState()
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
