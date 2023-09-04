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
	"runtime"
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

// Test final position and orientation are at approximately the expected values.
func testCartographerPosition(t *testing.T, svc slam.Service, useIMU bool, expectedComponentRef string) {
	var expectedPos r3.Vector
	var expectedOri *spatialmath.R4AA
	tolerancePos := 0.001
	toleranceOri := 0.001

	switch {
	case runtime.GOOS == "darwin" && useIMU:
		expectedPos = r3.Vector{X: 3.697475784009534, Y: 1.9629740734804884, Z: 0}
		expectedOri = &spatialmath.R4AA{
			RX:    0.9867946501586069,
			RY:    0.15958660525363744,
			RZ:    0.027720639277843195,
			Theta: 0.02201298037349276,
		}
		// integration_test.go:85: Position point: (3.697475784009534, 1.9629740734804884, -3.0247109197081865e-20)
		// integration_test.go:86: Expected '3.697475784009534' to be between '4.893611058933022' and '4.895611058933023' (but it wasn't)!
	case runtime.GOOS == "linux" && useIMU:
		expectedPos = r3.Vector{X: 8.9990012566792, Y: 3.38371853513753, Z: 0}
		expectedOri = &spatialmath.R4AA{
			RX:    0.9839756084582248,
			RY:    0.16541071768807708,
			RZ:    0.06656798354450472,
			Theta: 0.02229559831890859,
		}

	case runtime.GOOS == "darwin" && !useIMU:
		expectedPos = r3.Vector{X: 4.120375736261496, Y: 2.5213315508872665, Z: 0}
		expectedOri = &spatialmath.R4AA{
			RX:    0,
			RY:    0,
			RZ:    1,
			Theta: 0.0006629744894043836,
		}
	case runtime.GOOS == "linux" && !useIMU:
		expectedPos = r3.Vector{X: 3.36881455047323, Y: 4.130035158293928, Z: 0}
		expectedOri = &spatialmath.R4AA{
			RX:    0,
			RY:    0,
			RZ:    -1,
			Theta: 0.0010751949934010567,
		}
	}

	position, componentRef, err := svc.GetPosition(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, componentRef, test.ShouldEqual, expectedComponentRef)

	pos := position.Point()
	t.Logf("Position point: (%v, %v, %v)", pos.X, pos.Y, pos.Z)
	test.That(t, pos.X, test.ShouldBeBetween, expectedPos.X-tolerancePos, expectedPos.X+tolerancePos)
	test.That(t, pos.Y, test.ShouldBeBetween, expectedPos.Y-tolerancePos, expectedPos.Y+tolerancePos)
	test.That(t, pos.Z, test.ShouldBeBetween, expectedPos.Z-tolerancePos, expectedPos.Z+tolerancePos)

	ori := position.Orientation().AxisAngles()
	t.Logf("Position orientation: RX: %v, RY: %v, RZ: %v, Theta: %v", ori.RX, ori.RY, ori.RZ, ori.Theta)
	test.That(t, ori.RX, test.ShouldBeBetween, expectedOri.RX-toleranceOri, expectedOri.RX+toleranceOri)
	test.That(t, ori.RY, test.ShouldBeBetween, expectedOri.RY-toleranceOri, expectedOri.RY+toleranceOri)
	test.That(t, ori.RZ, test.ShouldBeBetween, expectedOri.RZ-toleranceOri, expectedOri.RZ+toleranceOri)
	test.That(t, ori.Theta, test.ShouldBeBetween, expectedOri.Theta-toleranceOri, expectedOri.Theta+toleranceOri)
}

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

// Saves cartographer's internal state in the data directory.
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

// testHelperCartographer is responsible for running a viam-cartographer process using the desired mock sensors. Once started it will
// waiting for all data to be processed by monitor sensor channels. After data has been fully processed, the endpoints Position,
// PointCloudMap, InternalState are evaluated and the process is closed out. The final internal state of cartographer is then returned.
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

	// Create config
	var timeTracker testhelper.TimeTracker
	attrCfg := &vcConfig.Config{
		ConfigParams: map[string]string{
			"mode": reflect.ValueOf(subAlgo).String(),
		},
		MapRateSec:            &mapRateSec,
		DataDirectory:         dataDirectory,
		IMUIntegrationEnabled: true,
	}

	// Add lidar component to config (required)
	lidarDone := make(chan struct{})
	lidarReadingInterval := time.Millisecond * 200
	attrCfg.Camera = map[string]string{"name": "stub_lidar"}
	timeTracker.LidarTime = time.Date(2021, 8, 15, 14, 30, 45, 1, time.UTC)

	// Add imu component to config (optional)
	imuDone := make(chan struct{})
	imuReadingInterval := time.Millisecond * 50
	if useIMU {
		attrCfg.MovementSensor = map[string]string{"name": "stub_imu"}
		timeTracker.ImuTime = time.Date(2021, 8, 15, 14, 30, 45, 1, time.UTC)
	}

	// Start Sensors
	timedLidar, err := testhelper.IntegrationTimedLidarSensor(t, attrCfg.Camera["name"],
		replaySensor, lidarReadingInterval, lidarDone, &timeTracker)
	test.That(t, err, test.ShouldBeNil)
	timedIMU, err := testhelper.IntegrationTimedIMUSensor(t, attrCfg.MovementSensor["name"],
		replaySensor, imuReadingInterval, imuDone, &timeTracker)
	test.That(t, err, test.ShouldBeNil)

	if !useIMU {
		test.That(t, timedIMU, test.ShouldBeNil)
	}

	// Start SLAM Service
	svc, err := testhelper.CreateIntegrationSLAMService(t, attrCfg, timedLidar, timedIMU, logger)
	test.That(t, err, test.ShouldBeNil)

	cSvc, ok := svc.(*viamcartographer.CartographerService)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, cSvc.SlamMode, test.ShouldEqual, expectedMode)

	// Waiting for sensor processes to finish sending data and for context to be canceled
	start := time.Now()
	ctx, cancelFunc := context.WithTimeout(context.Background(), time.Second*5)
	defer cancelFunc()

	// Wait for lidar sensor process to complete (required)
	if !utils.SelectContextOrWaitChan(ctx, lidarDone) {
		t.Logf("test duration %dms", time.Since(start).Milliseconds())
		test.That(t, errors.New("test timeout"), test.ShouldBeNil)
	}
	logger.Debug("lidar data finished")

	// Wait for IMU sensor process to complete (optional)
	if useIMU && !utils.SelectContextOrWaitChan(ctx, imuDone) {
		t.Logf("test duration %dms", time.Since(start).Milliseconds())
		test.That(t, errors.New("test timeout"), test.ShouldBeNil)
	}
	logger.Debug("imu data finished")

	// Test end points and retrieve internal state
	testCartographerPosition(t, svc, useIMU, attrCfg.Camera["name"])
	testCartographerMap(t, svc, cSvc.SlamMode == cartofacade.LocalizingMode)
	internalState, err := slam.GetInternalStateFull(context.Background(), svc)
	test.That(t, err, test.ShouldBeNil)
	logger.Debug("closing out service")

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

// TestIntegrationCartographer provides end-to-end testing of viam-cartographer using a combination of live vs. replay cameras
// and imu enabled mode.
func TestIntegrationCartographer(t *testing.T) {
	logger := golog.NewTestLogger(t)

	cases := []struct {
		description string
		replay      bool
		imuEnabled  bool
		mode        cartofacade.SlamMode
		subAlgo     viamcartographer.SubAlgo
	}{
		// Live sensor
		{
			description: "live sensor mapping mode 2D",
			replay:      false,
			imuEnabled:  false,
			mode:        cartofacade.MappingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "live sensor localizing mode 2D",
			replay:      false,
			imuEnabled:  false,
			mode:        cartofacade.LocalizingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "live sensor updating mode 2D",
			replay:      false,
			imuEnabled:  false,
			mode:        cartofacade.UpdatingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		// Replay sensor
		{
			description: "live sensor mapping mode 2D",
			replay:      true,
			imuEnabled:  false,
			mode:        cartofacade.MappingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "replay sensor localizing mode 2D",
			replay:      true,
			imuEnabled:  false,
			mode:        cartofacade.LocalizingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "replay sensor updating mode 2D",
			replay:      true,
			imuEnabled:  false,
			mode:        cartofacade.UpdatingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		// // Live + imu sensor
		{
			description: "live with imu sensor mapping mode 2D",
			replay:      false,
			imuEnabled:  true,
			mode:        cartofacade.MappingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "live with imu sensor localizing mode 2D",
			replay:      false,
			imuEnabled:  true,
			mode:        cartofacade.LocalizingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "live with imu sensor updating mode 2D",
			replay:      false,
			imuEnabled:  true,
			mode:        cartofacade.UpdatingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
	}

	// Loop over defined test cases, resetting the directories between slam sessions
	for _, tt := range cases {
		t.Run(tt.description, func(t *testing.T) {
			// Prep first run directory
			dataDirectory1, err := os.MkdirTemp("", "*")
			test.That(t, err, test.ShouldBeNil)
			defer func() {
				err := os.RemoveAll(dataDirectory1)
				test.That(t, err, test.ShouldBeNil)
			}()

			// Set mapRateSec for mapping mode
			mapRateSec := 1

			// Run mapping test
			internalState := testHelperCartographer(t, dataDirectory1, tt.subAlgo, logger, tt.replay, tt.imuEnabled, 1, cartofacade.MappingMode)

			// Return if in mapping mode as there are no further actions required
			if tt.mode == cartofacade.MappingMode {
				return
			}

			// Prep second run directory
			dataDirectory2, err := os.MkdirTemp("", "*")
			test.That(t, err, test.ShouldBeNil)
			defer func() {
				err := os.RemoveAll(dataDirectory2)
				test.That(t, err, test.ShouldBeNil)
			}()

			// Save internal state
			saveInternalState(t, internalState, dataDirectory2)

			// Run follow up updating or localizing test
			if tt.mode == cartofacade.LocalizingMode {
				mapRateSec = 0
			}
			testHelperCartographer(t, dataDirectory2, tt.subAlgo, logger, tt.replay, tt.imuEnabled, mapRateSec, tt.mode)
		})
	}
}
