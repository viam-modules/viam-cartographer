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

func testCartographerPosition(t *testing.T, svc slam.Service, expectedComponentRef string) {
	expectedPosOSX := r3.Vector{X: 155.7488316264227, Y: -90.25868252233964, Z: 0}
	expectedPosLinux := r3.Vector{X: 158.79903385710674, Y: -77.01514065531592, Z: 0}
	tolerancePos := 0.001

	expectedOriOSX := &spatialmath.R4AA{Theta: 1.5465081272043815, RX: 0, RY: 0, RZ: 1}
	expectedOriLinux := &spatialmath.R4AA{Theta: 0.3331667853231311, RX: 0, RY: 0, RZ: 1}
	toleranceOri := 0.001

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
		MapRateSec:    &mapRateSec,
		DataDirectory: dataDirectory,
		UseNewConfig:  "true",
	}

	done := make(chan struct{})
	sensorReadingInterval := time.Millisecond * 200
	timedSensor, err := testhelper.IntegrationLidarTimedSensor(t, attrCfg.Camera["name"], replaySensor, sensorReadingInterval, done)
	test.That(t, err, test.ShouldBeNil)
	svc, err := testhelper.CreateIntegrationSLAMService(t, attrCfg, timedSensor, logger)
	test.That(t, err, test.ShouldBeNil)

	start := time.Now()
	cSvc, ok := svc.(*viamcartographer.CartographerService)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, cSvc.SlamMode, test.ShouldEqual, expectedMode)
	ctx, cancelFunc := context.WithTimeout(context.Background(), time.Second*5)

	defer cancelFunc()

	// wait till all lidar readings have been read
	if !utils.SelectContextOrWaitChan(ctx, done) {
		test.That(t, errors.New("test timeout"), test.ShouldBeNil)
	}

	testCartographerPosition(t, svc, attrCfg.Camera["name"])
	testCartographerMap(t, svc, cSvc.SlamMode == cartofacade.LocalizingMode)

	internalState, err := slam.GetInternalStateFull(context.Background(), svc)
	test.That(t, err, test.ShouldBeNil)

	// Close out slam service
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)

	stop := time.Now()
	testDuration := stop.Sub(start)
	t.Logf("test duration %d", testDuration)

	// Check that a map was generated as the test has been running for more than the map rate msec
	mapsInDir, err := os.ReadDir(path.Join(dataDirectory, "internal_state"))
	test.That(t, err, test.ShouldBeNil)
	test.That(t, testDuration.Seconds(), test.ShouldBeGreaterThanOrEqualTo, time.Duration(*attrCfg.MapRateSec).Seconds())
	test.That(t, len(mapsInDir), test.ShouldBeGreaterThan, 0)

	// return the internal state so updating mode can be tested
	return internalState
}

func integrationtestHelperCartographer(t *testing.T, subAlgo viamcartographer.SubAlgo) {
	logger := golog.NewTestLogger(t)
	t.Run("live sensor mapping mode", func(t *testing.T) {
		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		testHelperCartographer(t, dataDirectory, subAlgo, logger, false, 1, cartofacade.MappingMode)
	})

	t.Run("replay sensor mapping mode", func(t *testing.T) {
		dataDirectory, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectory)
			test.That(t, err, test.ShouldBeNil)
		}()

		testHelperCartographer(t, dataDirectory, subAlgo, logger, true, 1, cartofacade.MappingMode)
	})

	t.Run("live sensor localizing mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run with replay sensor
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, 1, cartofacade.MappingMode)

		dataDirectoryLocalizing, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryLocalizing)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new datadir
		saveInternalState(t, internalState, dataDirectoryLocalizing)
		// localize on that internal state
		testHelperCartographer(t, dataDirectoryLocalizing, subAlgo, logger, false, 0, cartofacade.LocalizingMode)
	})

	t.Run("replay sensor localizing mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run with replay sensor
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, 1, cartofacade.MappingMode)

		dataDirectoryLocalizing, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryLocalizing)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new datadir
		saveInternalState(t, internalState, dataDirectoryLocalizing)
		// localize on that internal state
		testHelperCartographer(t, dataDirectoryLocalizing, subAlgo, logger, true, 0, cartofacade.LocalizingMode)
	})

	t.Run("live sensor updating mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, 1, cartofacade.MappingMode)

		dataDirectoryUpdating, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryUpdating)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new datadir
		saveInternalState(t, internalState, dataDirectoryUpdating)
		// update fromthat internal state
		testHelperCartographer(t, dataDirectoryUpdating, subAlgo, logger, false, 1, cartofacade.UpdatingMode)
	})

	t.Run("replay sensor updating mode", func(t *testing.T) {
		dataDirectoryMapping, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryMapping)
			test.That(t, err, test.ShouldBeNil)
		}()

		// do a mapping run
		internalState := testHelperCartographer(t, dataDirectoryMapping, subAlgo, logger, true, 1, cartofacade.MappingMode)

		dataDirectoryUpdating, err := os.MkdirTemp("", "*")
		test.That(t, err, test.ShouldBeNil)
		defer func() {
			err := os.RemoveAll(dataDirectoryUpdating)
			test.That(t, err, test.ShouldBeNil)
		}()

		// save the internal state of the mapping run to a new datadir
		saveInternalState(t, internalState, dataDirectoryUpdating)
		// update fromthat internal state
		testHelperCartographer(t, dataDirectoryUpdating, subAlgo, logger, true, 1, cartofacade.UpdatingMode)
	})
}

func TestCartographerIntegration2D(t *testing.T) {
	integrationtestHelperCartographer(t, viamcartographer.Dim2d)
}
