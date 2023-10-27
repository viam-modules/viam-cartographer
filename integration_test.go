// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user. It also runs integration tests
// that test the interaction with the core C++ viam-cartographer code and the Golang implementation of the
// cartographer slam service.
package viamcartographer_test

import (
	"bytes"
	"context"
	"os"
	"path/filepath"
	"reflect"
	"runtime"
	"strconv"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
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

const (
	defaultLidarTimeInterval          = 200
	defaultMovementSensorTimeInterval = 50
	testTimeout                       = 20 * time.Second
)

// Test final position and orientation are at approximately the expected values.
func testCartographerPosition(t *testing.T, svc slam.Service, useIMU bool, expectedComponentRef string) {
	var expectedPos r3.Vector
	var expectedOri *spatialmath.R4AA
	tolerancePos := 0.001
	toleranceOri := 0.001

	switch {
	case runtime.GOOS == "darwin" && !useIMU:
		expectedPos = r3.Vector{X: 1.9166854207566584, Y: 4.0381299349907644, Z: 0}
		expectedOri = &spatialmath.R4AA{
			RX:    0,
			RY:    0,
			RZ:    1,
			Theta: 0.0006629744894043836,
		}
	case runtime.GOOS == "linux" && !useIMU:
		expectedPos = r3.Vector{X: 7.507596391989648, Y: 3.193198802065579, Z: 0}
		expectedOri = &spatialmath.R4AA{
			RX:    0,
			RY:    0,
			RZ:    1,
			Theta: 0.001955831550003536,
		}

	case runtime.GOOS == "darwin" && useIMU:
		expectedPos = r3.Vector{X: 2.0505875736777743, Y: 5.223279102160396, Z: 0}
		expectedOri = &spatialmath.R4AA{
			RX:    0.9859881081149332,
			RY:    0.16566945009254153,
			RZ:    0.019521371928468846,
			Theta: 0.024957572547389457,
		}
	case runtime.GOOS == "linux" && useIMU:
		expectedPos = r3.Vector{X: 4.034335774857501, Y: 3.4162168550846896, Z: 0}
		expectedOri = &spatialmath.R4AA{
			RX:    0.9854659908213061,
			RY:    0.16580252152269065,
			RZ:    0.036963560316871265,
			Theta: 0.02496988133280465,
		}
	}

	position, componentRef, err := svc.Position(context.Background())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, componentRef, test.ShouldEqual, expectedComponentRef)

	pos := position.Point()
	t.Logf("Position point: (%v, %v, %v)", pos.X, pos.Y, pos.Z)
	test.That(t, pos.X, test.ShouldAlmostEqual, expectedPos.X, tolerancePos)
	test.That(t, pos.Y, test.ShouldAlmostEqual, expectedPos.Y, tolerancePos)
	test.That(t, pos.Z, test.ShouldAlmostEqual, expectedPos.Z, tolerancePos)

	ori := position.Orientation().AxisAngles()
	t.Logf("Position orientation: RX: %v, RY: %v, RZ: %v, Theta: %v", ori.RX, ori.RY, ori.RZ, ori.Theta)
	test.That(t, ori.RX, test.ShouldAlmostEqual, expectedOri.RX, toleranceOri)
	test.That(t, ori.RY, test.ShouldAlmostEqual, expectedOri.RY, toleranceOri)
	test.That(t, ori.RZ, test.ShouldAlmostEqual, expectedOri.RZ, toleranceOri)
	test.That(t, ori.Theta, test.ShouldAlmostEqual, expectedOri.Theta, toleranceOri)
}

// Checks the cartographer map and confirms there at least 100 map points.
func testCartographerMap(t *testing.T, svc slam.Service, localizationMode bool) {
	timestamp1, err := svc.LatestMapInfo(context.Background())
	test.That(t, err, test.ShouldBeNil)
	pcd, err := slam.PointCloudMapFull(context.Background(), svc)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, pcd, test.ShouldNotBeNil)
	timestamp2, err := svc.LatestMapInfo(context.Background())
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
func saveInternalState(t *testing.T, internalState []byte, dataDir string) string {
	timeStamp := time.Now().UTC()
	internalStateDir := filepath.Join(dataDir, "internal_state")
	err := os.Mkdir(internalStateDir, 0o755)
	test.That(t, err, test.ShouldBeNil)

	filename := filepath.Join(internalStateDir, "map_data_"+timeStamp.UTC().Format(testhelper.SlamTimeFormat)+".pbstream")
	err = os.WriteFile(filename, internalState, 0o644)
	test.That(t, err, test.ShouldBeNil)

	return filename
}

// testHelperCartographer is responsible for running a viam-cartographer process using the desired mock sensors. Once started it will
// wait for all data to be processed by monitor sensor channels. After data has been fully processed, the endpoints Position,
// PointCloudMap, InternalState are evaluated and the process is closed out. The final internal state of cartographer is then returned.
func testHelperCartographer(
	t *testing.T,
	existingMap string,
	subAlgo viamcartographer.SubAlgo,
	logger logging.Logger,
	replaySensor bool,
	online bool,
	useIMU bool,
	useOdometer bool,
	enableMapping bool,
	expectedMode cartofacade.SlamMode,
) []byte {
	termFunc := testhelper.InitTestCL(t, logger)
	defer termFunc()

	// Create config
	timeTracker := testhelper.TimeTracker{
		Mu: &sync.Mutex{},
	}

	attrCfg := &vcConfig.Config{
		ExistingMap:   existingMap,
		EnableMapping: &enableMapping,
		ConfigParams: map[string]string{
			"mode": reflect.ValueOf(subAlgo).String(),
		},
	}

	// Add lidar component to config (required)
	lidarDone := make(chan struct{})
	lidarReadingInterval := time.Millisecond * defaultLidarTimeInterval
	timeTracker.LidarTime = time.Date(2021, 8, 15, 14, 30, 45, 1, time.UTC)
	if !online {
		attrCfg.Camera = map[string]string{
			"name":              string(testhelper.LidarWithErroringFunctions),
			"data_frequency_hz": "0",
		}
	} else {
		attrCfg.Camera = map[string]string{
			"name":              string(testhelper.LidarWithErroringFunctions),
			"data_frequency_hz": strconv.Itoa(defaultLidarTimeInterval),
		}
	}

	movementSensorReadingInterval := time.Millisecond * defaultMovementSensorTimeInterval

	// Add imu component to config (optional)
	imuDone := make(chan struct{})
	if useIMU {
		if !online {
			attrCfg.MovementSensor = map[string]string{
				"name":              string(testhelper.MovementSensorWithErroringFunctions),
				"data_frequency_hz": "0",
			}
		} else {
			attrCfg.MovementSensor = map[string]string{
				"name":              string(testhelper.MovementSensorWithErroringFunctions),
				"data_frequency_hz": strconv.Itoa(defaultIMUTimeInterval),
			}
		}
		timeTracker.ImuTime = time.Date(2021, 8, 15, 14, 30, 45, 1, time.UTC)
	}

	// Start Sensors
	timedLidar, err := testhelper.IntegrationTimedLidar(t, attrCfg.Camera["name"],
		replaySensor, lidarReadingInterval, lidarDone, &timeTracker)
	test.That(t, err, test.ShouldBeNil)
	timedIMU, err := testhelper.IntegrationTimedIMU(t, attrCfg.MovementSensor["name"],
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

	// Wait for sensor processes to finish sending data and for context to be canceled
	start := time.Now().UTC()
	ctx, cancelFunc := context.WithTimeout(context.Background(), testTimeout)
	defer cancelFunc()

	finishedProcessingLidarData := utils.SelectContextOrWaitChan(ctx, lidarDone)
	t.Logf("lidar sensor process duration %dms (timeout = %dms)", time.Since(start).Milliseconds(), testTimeout.Milliseconds())
	test.That(t, finishedProcessingLidarData, test.ShouldBeTrue)

	if useIMU {
		finishedProcessingIMUData := utils.SelectContextOrWaitChan(ctx, imuDone)
		t.Logf("imu sensor process duration %dms (timeout = %dms)", time.Since(start).Milliseconds(), testTimeout.Milliseconds())
		test.That(t, finishedProcessingIMUData, test.ShouldBeTrue)
	}
	t.Logf("sensor processes have completed, all data has been ingested")

	// Test end points and retrieve internal state
	testCartographerPosition(t, svc, useIMU, attrCfg.Camera["name"])
	testCartographerMap(t, svc, cSvc.SlamMode == cartofacade.LocalizingMode)

	internalState, err := slam.InternalStateFull(context.Background(), svc)
	test.That(t, err, test.ShouldBeNil)
	logger.Debug("closing out service")

	// Close out slam service
	test.That(t, svc.Close(context.Background()), test.ShouldBeNil)
	testDuration := time.Since(start)
	t.Logf("test duration %dms", testDuration.Milliseconds())

	// return the internal state so updating mode can be tested
	return internalState
}

// TestIntegrationCartographer provides end-to-end testing of viam-cartographer using a combination of live vs. replay cameras
// and imu enabled mode.
func TestIntegrationCartographer(t *testing.T) {
	logger := logging.NewTestLogger(t)

	cases := []struct {
		description string
		online      bool
		replay      bool
		imuEnabled  bool
		mode        cartofacade.SlamMode
		subAlgo     viamcartographer.SubAlgo
	}{
		// Online sensor
		{
			description: "online sensor mapping mode 2D",
			online:      true,
			replay:      false,
			imuEnabled:  false,
			mode:        cartofacade.MappingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "online sensor localizing mode 2D",
			online:      true,
			replay:      false,
			imuEnabled:  false,
			mode:        cartofacade.LocalizingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "online sensor updating mode 2D",
			online:      true,
			replay:      false,
			imuEnabled:  false,
			mode:        cartofacade.UpdatingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		// Offline sensor
		{
			description: "offline sensor mapping mode 2D",
			online:      false,
			replay:      true,
			imuEnabled:  false,
			mode:        cartofacade.MappingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "offline sensor updating mode 2D",
			online:      false,
			replay:      true,
			imuEnabled:  false,
			mode:        cartofacade.UpdatingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		// Offline with imu sensor
		{
			description: "online with imu sensor mapping mode 2D",
			online:      true,
			replay:      true,
			imuEnabled:  true,
			mode:        cartofacade.MappingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "online with imu sensor localizing mode 2D",
			online:      true,
			replay:      true,
			imuEnabled:  true,
			mode:        cartofacade.LocalizingMode,
			subAlgo:     viamcartographer.Dim2d,
		},
		{
			description: "online with imu sensor updating mode 2D",
			online:      true,
			replay:      true,
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
			enableMapping := true

			// Run mapping test
			internalState := testHelperCartographer(
				t, "", tt.subAlgo, logger, tt.replay,
				tt.online, tt.imuEnabled, enableMapping, cartofacade.MappingMode,
			)

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
			existingMap := saveInternalState(t, internalState, dataDirectory2)
			test.That(t, existingMap, test.ShouldNotEqual, "")

			// Run follow up updating or localizing test
			if tt.mode == cartofacade.LocalizingMode {
				enableMapping = false
			}
			testHelperCartographer(
				t, existingMap, tt.subAlgo, logger, tt.replay,
				tt.online, tt.imuEnabled, enableMapping, tt.mode,
			)
		})
	}
}
