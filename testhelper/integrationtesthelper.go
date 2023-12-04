// Package testhelper provides test helpers which don't depend on viamcartographer
package testhelper

import (
	"bufio"
	"bytes"
	"context"
	"fmt"
	"os"
	"path"
	"reflect"
	"regexp"
	"runtime"
	"strconv"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	replaylidar "go.viam.com/rdk/components/camera/replaypcd"
	replaymovementsensor "go.viam.com/rdk/components/movementsensor/replay"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"go.viam.com/utils"
	"go.viam.com/utils/artifact"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	"github.com/viamrobotics/viam-cartographer/cartofacade"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/inject"
)

const (
	// NumPointClouds is the amount of mock lidar data saved in the mock_data/lidar slam artifact
	// directory used for integration tests.
	NumPointClouds = 10
	// NumIMUData is the amount of mock IMU data saved in the mock_data/imu/data.txt slam artifact
	// file used for integration tests.
	NumIMUData = 40
	// Path to slam mock data used for integration tests artifact path.
	// artifact.MustPath("viam-cartographer/mock_lidar").
	mockDataPath                      = "viam-cartographer/mock_data"
	sensorDataIngestionWaitTime       = 50 * time.Millisecond
	defaultLidarTimeInterval          = 200
	defaultMovementSensorTimeInterval = 50
	testTimeout                       = 20 * time.Second
)

var defaultTime = time.Time{}

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
		expectedPos = r3.Vector{X: 1.6456359928659154, Y: 7.359399690067484, Z: 0}
		expectedOri = &spatialmath.R4AA{
			RX:    0.9862302383600338,
			RY:    0.1635349032143911,
			RZ:    -0.02462219273278937,
			Theta: 0.025088490446011937,
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

	if localizationMode {
		test.That(t, timestamp1, test.ShouldResemble, timestamp2)
	} else {
		test.That(t, timestamp2.After(timestamp1), test.ShouldBeTrue)
	}

	pointcloud, _ := pointcloud.ReadPCD(bytes.NewReader(pcd))
	t.Logf("Pointcloud points: %v", pointcloud.Size())
	test.That(t, pointcloud.Size(), test.ShouldBeGreaterThanOrEqualTo, 100)
}

// timeTracker stores the current and next timestamps for both IMU and lidar. These are used to manually
// set the timestamp of each set of data being sent to cartographer and ensure proper ordering between them.
// This allows for consistent testing.
type timeTracker struct {
	lidarTime     time.Time
	nextLidarTime time.Time

	imuTime     time.Time
	nextImuTime time.Time

	lastLidarTime time.Time
	lastImuTime   time.Time

	mu *sync.Mutex
}

// integrationTimedLidar returns a mock timed lidar sensor
// or an error if preconditions to build the mock are not met.
// It validates that all required mock lidar reading files are able to be found.
// When the mock is called, it returns the next mock lidar reading, with the
// ReadingTime incremented by the sensorReadingInterval.
// The Replay sensor field of the mock readings will match the replay parameter.
// When the end of the mock lidar readings is reached, the done channel
// is written to once so the caller can detect when all lidar readings have been emitted
// from the mock. This is intended to match the same "end of dataset" behavior of a
// replay sensor.
// It is important to provide deterministic time information to cartographer to
// ensure test outputs of cartographer are deterministic.
func integrationTimedLidar(
	t *testing.T,
	lidar map[string]string,
	sensorReadingInterval time.Duration,
	done chan struct{},
	timeTracker *timeTracker,
) (s.TimedLidar, error) {
	// Check that the required amount of lidar data is present
	err := mockLidarReadingsValid()
	if err != nil {
		return nil, err
	}

	var i uint64

	dataFrequencyHz, err := strconv.Atoi(lidar["data_frequency_hz"])
	if err != nil {
		return nil, err
	}

	closed := false
	injectLidar := &inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return lidar["name"] }
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }
	injectLidar.TimedLidarReadingFunc = func(ctx context.Context) (s.TimedLidarReadingResponse, error) {
		defer timeTracker.mu.Unlock()
		/*
			Holds the process until for all necessary IMU data has been sent to cartographer. Only applicable
			when the IMU is present (timeTracker.NextImuTime has been defined) and is always true in the first iteration.
			This and the manual definition of timestamps allow for consistent results.
		*/
		for {
			timeTracker.mu.Lock()
			if timeTracker.imuTime == defaultTime {
				time.Sleep(sensorDataIngestionWaitTime)
				break
			}

			if i <= 1 || timeTracker.lidarTime.Sub(timeTracker.imuTime) <= 0 {
				time.Sleep(sensorDataIngestionWaitTime)
				break
			}
			timeTracker.mu.Unlock()
		}

		// Communicate that all lidar readings have been sent to cartographer or if the last IMU reading has been sent,
		// checks if LastLidarTime has been defined. If so, simulate endOfDataSet error.
		t.Logf("TimedLidarReading Mock i: %d, closed: %v, readingTime: %s\n", i, closed, timeTracker.lidarTime.String())
		if i >= NumPointClouds || timeTracker.lastImuTime != defaultTime {
			// Sends a signal to the integration sensor's done channel the first time end of dataset has been sent
			if !closed {
				done <- struct{}{}
				closed = true
				timeTracker.lastLidarTime = timeTracker.lidarTime
			}

			return s.TimedLidarReadingResponse{}, replaylidar.ErrEndOfDataset
		}

		// Get next lidar data
		resp, err := createTimedLidarReadingResponse(t, i, timeTracker)
		if err != nil {
			return resp, err
		}

		// Advance the data index and update time tracker (manual timestamps occurs here)
		i++
		timeTracker.lidarTime = timeTracker.lidarTime.Add(sensorReadingInterval)
		timeTracker.nextLidarTime = timeTracker.lidarTime.Add(sensorReadingInterval)

		return resp, nil
	}

	return injectLidar, nil
}

// IntegrationCartographer is responsible for running a viam-cartographer process using the desired mock sensors. Once started it will
// wait for all data to be processed by monitor sensor channels. After data has been fully processed, the endpoints Position,
// PointCloudMap, InternalState are evaluated and the process is closed out. The final internal state of cartographer is then returned.
func IntegrationCartographer(
	t *testing.T,
	existingMap string,
	subAlgo viamcartographer.SubAlgo,
	logger logging.Logger,
	online bool,
	useIMU bool,
	enableMapping bool,
	expectedMode cartofacade.SlamMode,
) []byte {
	termFunc := InitTestCL(t, logger)
	defer termFunc()

	// Create config
	timeTracker := timeTracker{
		mu: &sync.Mutex{},
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
	timeTracker.lidarTime = time.Date(2021, 8, 15, 14, 30, 45, 1, time.UTC)
	if !online {
		attrCfg.Camera = map[string]string{
			"name":              string(LidarWithErroringFunctions),
			"data_frequency_hz": "0",
		}
	} else {
		attrCfg.Camera = map[string]string{
			"name":              string(LidarWithErroringFunctions),
			"data_frequency_hz": strconv.Itoa(defaultLidarTimeInterval),
		}
	}

	movementSensorReadingInterval := time.Millisecond * defaultMovementSensorTimeInterval

	// Add imu component to config (optional)
	imuDone := make(chan struct{})
	if useIMU {
		if !online {
			attrCfg.MovementSensor = map[string]string{
				"name":              string(MovementSensorWithErroringFunctions),
				"data_frequency_hz": "0",
			}
		} else {
			attrCfg.MovementSensor = map[string]string{
				"name":              string(MovementSensorWithErroringFunctions),
				"data_frequency_hz": strconv.Itoa(defaultMovementSensorTimeInterval),
			}
		}
		timeTracker.imuTime = time.Date(2021, 8, 15, 14, 30, 45, 1, time.UTC)
	}

	// Start Sensors
	timedLidar, err := integrationTimedLidar(t, attrCfg.Camera,
		lidarReadingInterval, lidarDone, &timeTracker)
	test.That(t, err, test.ShouldBeNil)
	timedIMU, err := integrationTimedIMU(t, attrCfg.MovementSensor,
		movementSensorReadingInterval, imuDone, &timeTracker)
	test.That(t, err, test.ShouldBeNil)

	if !useIMU {
		test.That(t, timedIMU, test.ShouldBeNil)
	}

	// Start SLAM Service
	svc, err := CreateIntegrationSLAMService(t, attrCfg, timedLidar, timedIMU, logger)
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

// integrationTimedIMU returns a mock timed IMU sensor.
// When the mock is called, it returns the next mock IMU readings, with the
// ReadingTime incremented by the sensorReadingInterval.
// The Replay sensor field of the mock readings will match the replay parameter.
// When the end of the mock IMU readings is reached, the done channel
// is written to once so the caller can detect when all IMU readings have been emitted
// from the mock. This is intended to match the same "end of dataset" behavior of a
// replay sensor.
// It is important to provide deterministic time information to cartographer to
// ensure test outputs of cartographer are deterministic.
func integrationTimedIMU(
	t *testing.T,
	movementSensor map[string]string,
	sensorReadingInterval time.Duration,
	done chan struct{},
	timeTracker *timeTracker,
) (s.TimedMovementSensor, error) {
	// Return nil if IMU is not requested
	if movementSensor["name"] == "" {
		return nil, nil
	}

	// Check that the required amount of IMU data is present and create a mock dataset from provided mock data artifact file.
	mockDataset, err := mockIMUReadingsValid(t)
	if err != nil {
		return nil, err
	}

	dataFrequencyHz, err := strconv.Atoi(movementSensor["data_frequency_hz"])
	if err != nil {
		return nil, err
	}

	var i uint64
	closed := false
	injectIMU := &inject.TimedMovementSensor{}
	injectIMU.NameFunc = func() string { return movementSensor["name"] }
	injectIMU.DataFrequencyHzFunc = func() int { return dataFrequencyHz }
	injectIMU.TimedMovementSensorReadingFunc = func(ctx context.Context) (s.TimedMovementSensorReadingResponse, error) {
		defer timeTracker.mu.Unlock()
		/*
			Holds the process until for all necessary lidar data has been sent to cartographer. Is always
			true in the first iteration. This and the manual definition of timestamps allow for consistent
			results.
		*/
		for {
			timeTracker.mu.Lock()
			if i == 0 || timeTracker.imuTime.Sub(timeTracker.nextLidarTime) < 0 {
				time.Sleep(sensorDataIngestionWaitTime)
				break
			}
			timeTracker.mu.Unlock()
		}

		// Communicate that all desired IMU readings have been sent or to cartographer or if the last lidar reading
		// has been sent by, checks if lastLidarTime has been defined. If so, simulate endOfDataSet error.
		t.Logf("TimedMovementSensorReading Mock i: %d, closed: %v, readingTime: %s\n", i, closed, timeTracker.imuTime.String())
		if int(i) >= len(mockDataset) || timeTracker.lastLidarTime != defaultTime {
			// Sends a signal to the integration sensor's done channel the first time end of dataset has been sent
			if !closed {
				done <- struct{}{}
				closed = true
				timeTracker.lastImuTime = timeTracker.imuTime
			}
			return s.TimedMovementSensorReadingResponse{}, replaymovementsensor.ErrEndOfDataset
		}

		// Get next IMU data
		resp, err := createTimedMovementSensorReadingResponse(t, mockDataset[i], timeTracker)
		if err != nil {
			return resp, err
		}

		// Advance the data index and update time tracker (manual timestamps occurs here)
		i++
		timeTracker.imuTime = timeTracker.imuTime.Add(sensorReadingInterval)
		timeTracker.nextImuTime = timeTracker.imuTime.Add(sensorReadingInterval)

		return resp, nil
	}
	injectIMU.PropertiesFunc = func() s.MovementSensorProperties {
		return s.MovementSensorProperties{
			IMUSupported: true,
		}
	}

	return injectIMU, nil
}

func createTimedLidarReadingResponse(t *testing.T, i uint64, timeTracker *timeTracker,
) (s.TimedLidarReadingResponse, error) {
	file, err := os.Open(artifact.MustPath(mockDataPath + "/lidar/" + strconv.FormatUint(i, 10) + ".pcd"))
	if err != nil {
		t.Error("TEST FAILED TimedLidarReading Mock failed to open pcd file")
		return s.TimedLidarReadingResponse{}, err
	}
	readingPc, err := pointcloud.ReadPCD(file)
	if err != nil {
		t.Error("TEST FAILED TimedLidarReading Mock failed to read pcd")
		return s.TimedLidarReadingResponse{}, err
	}

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary)
	if err != nil {
		t.Error("TEST FAILED TimedLidarReading Mock failed to parse pcd")
		return s.TimedLidarReadingResponse{}, err
	}

	resp := s.TimedLidarReadingResponse{
		Reading:     buf.Bytes(),
		ReadingTime: timeTracker.lidarTime,
	}
	return resp, nil
}

func createTimedMovementSensorReadingResponse(t *testing.T, line string, timeTracker *timeTracker,
) (s.TimedMovementSensorReadingResponse, error) {
	re := regexp.MustCompile(`[-+]?\d*\.?\d+`)
	matches := re.FindAllString(line, -1)

	linAccX, err1 := strconv.ParseFloat(matches[0], 64)
	linAccY, err2 := strconv.ParseFloat(matches[1], 64)
	linAccZ, err3 := strconv.ParseFloat(matches[2], 64)
	if err1 != nil || err2 != nil || err3 != nil {
		t.Error("TEST FAILED TimedMovementSensorReading Mock failed to parse linear acceleration")
		return s.TimedMovementSensorReadingResponse{}, errors.New("error parsing linear acceleration from file")
	}
	linAcc := r3.Vector{X: linAccX, Y: linAccY, Z: linAccZ}

	angVelX, err1 := strconv.ParseFloat(matches[3], 64)
	angVelY, err2 := strconv.ParseFloat(matches[4], 64)
	angVelZ, err3 := strconv.ParseFloat(matches[5], 64)
	if err1 != nil || err2 != nil || err3 != nil {
		t.Error("TEST FAILED TimedMovementSensorReading Mock failed to parse angular velocity")
		return s.TimedMovementSensorReadingResponse{}, errors.New("error parsing angular velocity from file")
	}
	angVel := spatialmath.AngularVelocity{X: angVelX, Y: angVelY, Z: angVelZ}

	resp := s.TimedMovementSensorReadingResponse{
		TimedIMUResponse: &s.TimedIMUReadingResponse{
			LinearAcceleration: linAcc,
			AngularVelocity:    angVel,
			ReadingTime:        timeTracker.imuTime,
		},
	}
	return resp, nil
}

func mockLidarReadingsValid() error {
	dirEntries, err := os.ReadDir(artifact.MustPath(mockDataPath + "/lidar"))
	if err != nil {
		return err
	}

	var files []string
	for _, f := range dirEntries {
		if !f.IsDir() {
			files = append(files, f.Name())
		}
	}
	if len(files) < NumPointClouds {
		return errors.Errorf("expected at least %v lidar readings for integration test", NumPointClouds)
	}
	for i := 0; i < NumPointClouds; i++ {
		found := false
		expectedFile := fmt.Sprintf("%d.pcd", i)
		for _, file := range files {
			if file == expectedFile {
				found = true
				break
			}
		}

		if !found {
			return errors.Errorf("expected %s to exist for integration test", path.Join(mockDataPath+"/lidar", expectedFile))
		}
	}
	return nil
}

func mockIMUReadingsValid(t *testing.T) ([]string, error) {
	file, err := os.Open(artifact.MustPath(mockDataPath + "/imu/data.txt"))
	if err != nil {
		t.Error("TEST FAILED TimedIMUSensorReading Mock failed to open data file")
		return []string{}, err
	}

	mockDatasetScanner := bufio.NewScanner(file)
	mockDatasetScanner.Split(bufio.ScanLines)
	var mockDataset []string

	for mockDatasetScanner.Scan() {
		mockDataset = append(mockDataset, mockDatasetScanner.Text())
	}

	if len(mockDataset) < NumIMUData {
		return []string{}, errors.Errorf("expected at least %v imu readings for integration test", NumIMUData)
	}
	return mockDataset, nil
}
