// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user. It also runs integration tests
// that test the interaction with the core C++ viam-cartographer code and the Golang implementation of the
// cartographer slam service.
package viamcartographer_test

import (
	"bytes"
	"context"
	"errors"
	"os"
	"path"
	"path/filepath"
	"reflect"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"go.viam.com/utils"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	"github.com/viamrobotics/viam-cartographer/cartofacade"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	"github.com/viamrobotics/viam-cartographer/testhelper"
)

// Checks the cartographer map and confirms there at least 100 map points.
func testCartographerMap(t *testing.T, svc slam.Service, localizationMode bool) {
	timestamp1, err := svc.GetLatestMapInfo(context.Background())
	test.That(t, err, test.ShouldBeNil)
	pcd, err := slam.GetPointCloudMapFull(context.Background(), svc)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pcd, test.ShouldNotBeNil)
	timestamp2, err := svc.GetLatestMapInfo(context.Background())
	test.That(t, err, test.ShouldBeNil)

	if localizationMode == true {
		test.That(t, timestamp1, test.ShouldResemble, timestamp2)
	} else {
		test.That(t, timestamp2.After(timestamp1), test.ShouldBeTrue)
	}

	pointcloud, _ := pointcloud.ReadPCD(bytes.NewReader(pcd))
	t.Logf("Pointcloud points: %v", pointcloud.Size())
	test.That(t, pointcloud.Size(), test.ShouldBeGreaterThanOrEqualTo, 100)
}

func testCartographerPosition(t *testing.T, svc slam.Service, useIMU bool, mode cartofacade.SlamMode, expectedComponentRef string) {
	var expectedPosOSX r3.Vector
	var expectedPosLinux r3.Vector
	var expectedOriOSX *spatialmath.R4AA
	var expectedOriLinux *spatialmath.R4AA
	tolerancePos := 0.001
	toleranceOri := 0.001

	switch {
	case useIMU && mode == cartofacade.UpdatingMode:
		expectedPosOSX = r3.Vector{X: 5.882838701736658, Y: 3.071019233988039, Z: 0}
		expectedOriOSX = &spatialmath.R4AA{Theta: 0.023418952088469582, RX: 0.9847737291364351, RY: 0.17332572441345112, RZ: -0.013375188195744212}

		expectedPosLinux = r3.Vector{X: 33.36424739867359, Y: -15.892546207753742, Z: -1.7763568394002505e-15}
		expectedOriLinux = &spatialmath.R4AA{Theta: 1.6301758733667822, RX: 0.9252197096950275, RY: 0.04712768411234466, RZ: 0.3764936522466959}
	case useIMU && mode != cartofacade.UpdatingMode:
		expectedPosOSX = r3.Vector{X: 4.7290456742637685, Y: 3.840642095845822, Z: 0}
		expectedOriOSX = &spatialmath.R4AA{Theta: 0.02342781010456736, RX: 0.9843120830417524, RY: 0.1737669232094075, RZ: 0.030574165178177792}

		expectedPosLinux = r3.Vector{X: 33.36424739867359, Y: -15.892546207753742, Z: -1.7763568394002505e-15}
		expectedOriLinux = &spatialmath.R4AA{Theta: 1.6301758733667822, RX: 0.9252197096950275, RY: 0.04712768411234466, RZ: 0.3764936522466959}
	case !useIMU && mode == cartofacade.UpdatingMode:
		expectedPosOSX = r3.Vector{X: 3.558142186397387, Y: 3.7690587022387874, Z: 0}
		expectedOriOSX = &spatialmath.R4AA{Theta: 0.00228885684885118, RX: 0, RY: 0, RZ: 1}

		expectedPosLinux = r3.Vector{X: 158.79903385710674, Y: -77.01514065531592, Z: 0}
		expectedOriLinux = &spatialmath.R4AA{Theta: 0.3331667853231311, RX: 0, RY: 0, RZ: 1}
	case !useIMU && mode != cartofacade.UpdatingMode:
		expectedPosOSX = r3.Vector{X: 3.5169344230934447, Y: 3.519425001143934, Z: 0}
		expectedOriOSX = &spatialmath.R4AA{Theta: 0.0003758992914140011, RX: 0, RY: 0, RZ: 1}

		expectedPosLinux = r3.Vector{X: 158.79903385710674, Y: -77.01514065531592, Z: 0}
		expectedOriLinux = &spatialmath.R4AA{Theta: 0.3331667853231311, RX: 0, RY: 0, RZ: 1}
	}

	position, componentRef, err := svc.GetPosition(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, componentRef, test.ShouldEqual, expectedComponentRef)

	actualPos := position.Point()
	t.Logf("Position point: (%v, %v, %v)", actualPos.X, actualPos.Y, actualPos.Z)
	// https://viam.atlassian.net/browse/RSDK-3866
	// mac
	if actualPos.X > expectedPosOSX.X-tolerancePos && actualPos.X < expectedPosOSX.X+tolerancePos {
		test.That(t, actualPos.Y, test.ShouldBeBetween, expectedPosOSX.Y-tolerancePos, expectedPosOSX.Y+tolerancePos)
		test.That(t, actualPos.Z, test.ShouldBeBetween, expectedPosOSX.Z-tolerancePos, expectedPosOSX.Z+tolerancePos)
		// linux
	} else if actualPos.X > expectedPosLinux.X-tolerancePos && actualPos.X < expectedPosLinux.X+tolerancePos {
		test.That(t, actualPos.Y, test.ShouldBeBetween, expectedPosLinux.Y-tolerancePos, expectedPosLinux.Y+tolerancePos)
		test.That(t, actualPos.Z, test.ShouldBeBetween, expectedPosLinux.Z-tolerancePos, expectedPosLinux.Z+tolerancePos)
	} else {
		t.Error("TEST FAILED Position is outside of expected platform range")
	}

	actualOri := position.Orientation().AxisAngles()
	t.Logf("Position orientation: RX: %v, RY: %v, RZ: %v, Theta: %v", actualOri.RX, actualOri.RY, actualOri.RZ, actualOri.Theta)

	if actualOri.Theta > expectedOriOSX.Theta-toleranceOri && actualOri.Theta < expectedOriOSX.Theta+toleranceOri {
		test.That(t, actualOri.RX, test.ShouldBeBetween, expectedOriOSX.RX-toleranceOri, expectedOriOSX.RX+toleranceOri)
		test.That(t, actualOri.RY, test.ShouldBeBetween, expectedOriOSX.RY-toleranceOri, expectedOriOSX.RY+toleranceOri)
		test.That(t, actualOri.Theta, test.ShouldBeBetween, expectedOriOSX.Theta-toleranceOri, expectedOriOSX.Theta+toleranceOri)
	} else if actualOri.Theta > expectedOriLinux.Theta-toleranceOri && actualOri.Theta < expectedOriLinux.Theta+toleranceOri {
		test.That(t, actualOri.RX, test.ShouldBeBetween, expectedOriLinux.RX-toleranceOri, expectedOriLinux.RX+toleranceOri)
		test.That(t, actualOri.RY, test.ShouldBeBetween, expectedOriLinux.RY-toleranceOri, expectedOriLinux.RY+toleranceOri)
		test.That(t, actualOri.RZ, test.ShouldBeBetween, expectedOriLinux.RZ-toleranceOri, expectedOriLinux.RZ+toleranceOri)
	} else {
		t.Error("TEST FAILED Orientation is outside of expected platform range")
	}
}

func saveInternalState(t *testing.T, internalState []byte, dataDir string) {
	timeStamp := time.Now()
	internalStateDir := filepath.Join(dataDir, "internal_state")
	if err := os.Mkdir(internalStateDir, 0o755); err != nil {
		t.Error("TEST FAILED failed to create test internal state directory")
	}
	filename := filepath.Join(internalStateDir, "map_data_"+timeStamp.UTC().Format(testhelper.SlamTimeFormat)+".pbstream")
	if err := os.WriteFile(filename, internalState, 0o644); err != nil {
		t.Error("TEST FAILED failed to write test internal state")
	}
}

func testHelperCartographer(
	t *testing.T,
	dataDirectory string,
	subAlgo viamcartographer.SubAlgo,
	logger golog.Logger,
	replaySensor bool,
	useIMU bool,
	mapRateSec int,
	expectedMode cartofacade.SlamMode,
) []byte {
	termFunc := testhelper.InitTestCL(t, logger)
	defer termFunc()

	attrCfg := &vcConfig.Config{
		Camera: map[string]string{"name": "stub_lidar"},
		ConfigParams: map[string]string{
			"mode": reflect.ValueOf(subAlgo).String(),
		},
		MapRateSec:            &mapRateSec,
		DataDirectory:         dataDirectory,
		IMUIntegrationEnabled: true,
	}
	if useIMU {
		attrCfg.MovementSensor = map[string]string{"name": "stub_imu"}
	}

	lidarDone := make(chan struct{})
	imuDone := make(chan struct{})
	lidarReadingInterval := time.Millisecond * 200
	imuReadingInterval := time.Millisecond * 50
	timedLidar, err := testhelper.IntegrationTimedLidarSensor(t, attrCfg.Camera["name"], replaySensor, lidarReadingInterval, lidarDone)
	test.That(t, err, test.ShouldBeNil)
	timedIMU, err := testhelper.IntegrationTimedIMUSensor(t, attrCfg.MovementSensor["name"], replaySensor, imuReadingInterval, imuDone)
	test.That(t, err, test.ShouldBeNil)
	if !useIMU {
		test.That(t, timedIMU, test.ShouldBeNil)
	}
	svc, err := testhelper.CreateIntegrationSLAMService(t, attrCfg, timedLidar, timedIMU, logger)
	test.That(t, err, test.ShouldBeNil)

	start := time.Now()
	cSvc, ok := svc.(*viamcartographer.CartographerService)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, cSvc.SlamMode, test.ShouldEqual, expectedMode)
	ctx, cancelFunc := context.WithTimeout(context.Background(), time.Second*5)

	defer cancelFunc()
	// wait till all lidar readings have been read
	if !utils.SelectContextOrWaitChan(ctx, lidarDone) {
		t.Logf("test duration %dms", time.Since(start).Milliseconds())
		test.That(t, errors.New("test timeout"), test.ShouldBeNil)
	}

	if useIMU {
		if !utils.SelectContextOrWaitChan(ctx, imuDone) {
			t.Logf("test duration %dms", time.Since(start).Milliseconds())
			test.That(t, errors.New("test timeout"), test.ShouldBeNil)
		}
	}

	testCartographerPosition(t, svc, useIMU, expectedMode, attrCfg.Camera["name"])
	testCartographerMap(t, svc, cSvc.SlamMode == cartofacade.LocalizingMode)

	internalState, err := slam.GetInternalStateFull(context.Background(), svc)
	test.That(t, err, test.ShouldBeNil)

	// Close out slam service
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	testDuration := time.Since(start)
	t.Logf("test duration %dms", testDuration.Milliseconds())

	// Check that a map was generated as the test has been running for more than the map rate msec
	mapsInDir, err := os.ReadDir(path.Join(dataDirectory, "internal_state"))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, testDuration.Seconds(), test.ShouldBeGreaterThanOrEqualTo, time.Duration(*attrCfg.MapRateSec).Seconds())
	test.That(t, len(mapsInDir), test.ShouldBeGreaterThan, 0)

	// return the internal state so updating mode can be tested
	return internalState
}

func integrationTestHelperCartographer(t *testing.T, subAlgo viamcartographer.SubAlgo) {
	logger := golog.NewTestLogger(t)

	t.Run("live sensor mapping mode", func(t *testing.T) {
		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		testHelperCartographer(t, dataDirectory, subAlgo, logger, false, false, 1, cartofacade.MappingMode)
	})

	t.Run("live sensor localizing mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run with replay sensor
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, false, 1, cartofacade.MappingMode)

		dataDirectoryLocalizing, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryLocalizing)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new data_dir
		saveInternalState(t, internalState, dataDirectoryLocalizing)
		// localize on that internal state
		testHelperCartographer(t, dataDirectoryLocalizing, subAlgo, logger, false, false, 0, cartofacade.LocalizingMode)
	})

	t.Run("live sensor updating mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, false, 1, cartofacade.MappingMode)

		dataDirectoryUpdating, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryUpdating)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new data_dir
		saveInternalState(t, internalState, dataDirectoryUpdating)
		// update that internal state
		testHelperCartographer(t, dataDirectoryUpdating, subAlgo, logger, false, false, 1, cartofacade.UpdatingMode)
	})
}

func integrationTestHelperCartographerWithIMU(t *testing.T, subAlgo viamcartographer.SubAlgo) {
	logger := golog.NewTestLogger(t)

	t.Run("live sensor mapping mode", func(t *testing.T) {
		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		testHelperCartographer(t, dataDirectory, subAlgo, logger, false, true, 1, cartofacade.MappingMode)
	})

	t.Run("live sensor localizing mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run with replay sensor
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, true, 1, cartofacade.MappingMode)

		dataDirectoryLocalizing, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryLocalizing)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new data_dir
		saveInternalState(t, internalState, dataDirectoryLocalizing)
		// localize on that internal state
		testHelperCartographer(t, dataDirectoryLocalizing, subAlgo, logger, false, true, 0, cartofacade.LocalizingMode)
	})

	t.Run("live sensor updating mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, true, 1, cartofacade.MappingMode)

		dataDirectoryUpdating, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryUpdating)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new data_dir
		saveInternalState(t, internalState, dataDirectoryUpdating)
		// update from that internal state
		testHelperCartographer(t, dataDirectoryUpdating, subAlgo, logger, false, true, 1, cartofacade.UpdatingMode)
	})
}

func integrationTestHelperCartographerReplay(t *testing.T, subAlgo viamcartographer.SubAlgo) {
	logger := golog.NewTestLogger(t)

	t.Run("replay sensor mapping mode", func(t *testing.T) {
		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		testHelperCartographer(t, dataDirectory, subAlgo, logger, true, false, 1, cartofacade.MappingMode)
	})

	t.Run("replay sensor localizing mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run with replay sensor
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, false, 1, cartofacade.MappingMode)

		dataDirectoryLocalizing, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryLocalizing)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new data_dir
		saveInternalState(t, internalState, dataDirectoryLocalizing)
		// localize on that internal state
		testHelperCartographer(t, dataDirectoryLocalizing, subAlgo, logger, true, false, 0, cartofacade.LocalizingMode)
	})

	t.Run("replay sensor updating mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, false, 1, cartofacade.MappingMode)

		dataDirectoryUpdating, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryUpdating)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new data_dir
		saveInternalState(t, internalState, dataDirectoryUpdating)
		// update from that internal state
		testHelperCartographer(t, dataDirectoryUpdating, subAlgo, logger, true, false, 1, cartofacade.UpdatingMode)
	})
}

func TestCartographerIntegration2D(t *testing.T) {
	integrationTestHelperCartographer(t, viamcartographer.Dim2d)
}

func TestCartographerIntegrationWithIMU2D(t *testing.T) {
	integrationTestHelperCartographerWithIMU(t, viamcartographer.Dim2d)
}

func TestCartographerIntegrationReplay2D(t *testing.T) {
	integrationTestHelperCartographerReplay(t, viamcartographer.Dim2d)
}
