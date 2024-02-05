// Package testhelper provides test helpers which don't depend on viamcartographer
package testhelper

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path"
	"reflect"
	"runtime"
	"strconv"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
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
	// NumPointCloudFiles is the number of mock lidar data files we want to use for the
	// integration tests. The files are stored in the mock_data/lidar slam artifact directory.
	NumPointCloudFiles = 10
	// NumMovementSensorData is the amount of mock movement sensor data we want to use for the
	// integration tests. The data is stored in the mock_data/movement_sensor/data.txt slam artifact file.
	NumMovementSensorData = 40
	// mockDataPath is the path to slam mock data used for integration tests artifact path.
	mockDataPath                      = "viam-cartographer/mock_data"
	sensorDataIngestionWaitTime       = 50 * time.Millisecond
	defaultLidarTimeInterval          = 200 * time.Millisecond
	defaultMovementSensorTimeInterval = 50 * time.Millisecond
	testTimeout                       = 20 * time.Second
	darwin                            = "darwin"
	linux                             = "linux"
)

type dataTime struct {
	Seconds int `json:"seconds"`
	Nanos   int `json:"nanos"`
}

type coordinate struct {
	Latitude  float64 `json:"latitude"`
	Longitude float64 `json:"longitude"`
}

type angularVelocity struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type linearAcceleration struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type orientation struct {
	Ox    float64 `json:"o_x"`
	Oy    float64 `json:"o_y"`
	Oz    float64 `json:"o_z"`
	Theta float64 `json:"theta"`
}

type angVelData struct {
	MetaDataIndex int             `json:"MetadataIndex"`
	TimeReceived  dataTime        `json:"TimeReceived"`
	TimeRequested dataTime        `json:"TimeRequested"`
	AngVel        angularVelocity `json:"angular_velocity"`
}

type linAccData struct {
	MetaDataIndex int                `json:"MetadataIndex"`
	TimeReceived  dataTime           `json:"TimeReceived"`
	TimeRequested dataTime           `json:"TimeRequested"`
	LinAcc        linearAcceleration `json:"linear_acceleration"`
}

type orientationData struct {
	MetaDataIndex int         `json:"MetadataIndex"`
	TimeReceived  dataTime    `json:"TimeReceived"`
	TimeRequested dataTime    `json:"TimeRequested"`
	Orientation   orientation `json:"orientation"`
}

type posData struct {
	MetaDataIndex int        `json:"MetadataIndex"`
	TimeReceived  dataTime   `json:"TimeReceived"`
	TimeRequested dataTime   `json:"TimeRequested"`
	AltitudeM     int        `json:"altitude_m"`
	Coordinate    coordinate `json:"coordinate"`
}

type movementSensorData struct {
	AngVelData      []angVelData      `json:"AngVelData"`
	LinAccData      []linAccData      `json:"LinAccData"`
	OrientationData []orientationData `json:"OrientationData"`
	PosData         []posData         `json:"PosData"`
}

// Test final position and orientation are at approximately the expected values.
func testCartographerPosition(t *testing.T, svc slam.Service, useIMU bool,
	useOdometer bool, expectedComponentRef string,
) {
	var expectedPos r3.Vector
	var expectedOri *spatialmath.R4AA
	tolerancePos := 0.001
	toleranceOri := 0.001

	// Online && Offline run with lidar only
	if !useIMU && !useOdometer {
		switch {
		case runtime.GOOS == darwin:
			expectedPos = r3.Vector{X: 1.9166854207566584, Y: 4.0381299349907644, Z: 0}
			expectedOri = &spatialmath.R4AA{
				RX:    0,
				RY:    0,
				RZ:    1,
				Theta: 0.0006629744894043836,
			}
		case runtime.GOOS == linux:
			expectedPos = r3.Vector{X: -5.149871228951607, Y: -1.824249792681155, Z: 0}
			expectedOri = &spatialmath.R4AA{
				RX:    0,
				RY:    0,
				RZ:    1,
				Theta: 0.0023832043348390474,
			}
		}
	}

	// Online run with lidar + imu
	if useIMU && !useOdometer {
		switch {
		case runtime.GOOS == darwin:
			expectedPos = r3.Vector{X: 1.6456359928659154, Y: 7.359399690067484, Z: 0}
			expectedOri = &spatialmath.R4AA{
				RX:    0.9862302383600338,
				RY:    0.1635349032143911,
				RZ:    -0.02462219273278937,
				Theta: 0.025088490446011937,
			}
		case runtime.GOOS == linux:
			expectedPos = r3.Vector{X: -7.219897923472782, Y: -1.0853619028673704, Z: 0}
			expectedOri = &spatialmath.R4AA{
				RX:    0.9992897370543805,
				RY:    -0.004423527448316455,
				RZ:    0.0374226378372907,
				Theta: 0.08557355172549885,
			}
		}
	}

	// Online run with lidar + odometer
	if !useIMU && useOdometer {
		switch {
		case runtime.GOOS == darwin:
			expectedPos = r3.Vector{X: 1.6456359928659154, Y: 7.359399690067484, Z: 0}
			expectedOri = &spatialmath.R4AA{
				RX:    0.9862302383600338,
				RY:    0.1635349032143911,
				RZ:    -0.02462219273278937,
				Theta: 0.025088490446011937,
			}
		case runtime.GOOS == linux:
			expectedPos = r3.Vector{X: -5.149871228951607, Y: -1.824249792681155, Z: 0}
			expectedOri = &spatialmath.R4AA{
				RX:    0,
				RY:    0,
				RZ:    1,
				Theta: 0.0023832043348390474,
			}
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

	pointcloud, err := pointcloud.ReadPCD(bytes.NewReader(pcd))
	test.That(t, err, test.ShouldBeNil)
	t.Logf("Pointcloud points: %v", pointcloud.Size())
	test.That(t, pointcloud.Size(), test.ShouldBeGreaterThanOrEqualTo, 100)
}

// timeTracker stores the current and next timestamps for both the movement sensor and the lidar.
// These are used to manually set the timestamp of each set of data being sent to cartographer
// and ensure proper ordering between them. This allows for consistent testing.
type timeTracker struct {
	lidarTime     time.Time
	nextLidarTime time.Time

	movementSensorTime time.Time

	lidarDone          bool
	movementSensorDone bool

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
	useMovementSensor bool,
) (s.TimedLidar, error) {
	// Check that the required amount of lidar data is present
	if err := mockLidarReadingsValid(); err != nil {
		return nil, err
	}

	var i uint64

	dataFrequencyHz, err := strconv.Atoi(lidar["data_frequency_hz"])
	if err != nil {
		return nil, err
	}

	injectLidar := &inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return lidar["name"] }
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }
	injectLidar.TimedLidarReadingFunc = func(ctx context.Context) (s.TimedLidarReadingResponse, error) {
		defer timeTracker.mu.Unlock()

		if useMovementSensor {
			// Holds the process until movement sensor data has been sent to cartographer. Is always true
			// in the first iteration. This and the manual definition of timestamps allow for consistent results.
			for {
				timeTracker.mu.Lock()
				if !timeTracker.lidarTime.After(timeTracker.movementSensorTime) {
					break
				}
				timeTracker.mu.Unlock()
			}
		} else {
			timeTracker.mu.Lock()
		}
		time.Sleep(sensorDataIngestionWaitTime)

		// Return the ErrEndOfDataset if all lidar readings have been sent to cartographer or if the
		// movement sensor is done.
		if i >= NumPointCloudFiles || timeTracker.movementSensorDone {
			// Sends a signal to the integration sensor's done channel the first time end of dataset has been sent
			if !timeTracker.lidarDone {
				done <- struct{}{}
				timeTracker.lidarDone = true
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
// wait for all data to be processed by monitoring sensor channels. After data has been fully processed, the endpoints Position,
// PointCloudMap, and InternalState are evaluated, and the process is closed out. The final internal state of cartographer is then returned.
func IntegrationCartographer(
	t *testing.T,
	existingMap string,
	subAlgo viamcartographer.SubAlgo,
	logger logging.Logger,
	online bool,
	useIMU bool,
	useOdometer bool,
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
	timeTracker.lidarTime = time.Date(2021, 8, 15, 14, 30, 45, 1, time.UTC)

	// We're using LidarWithErroringFunctions as a placeholder for deps. We're defining and
	// using the injection lidar to overwrite this lidar when we create the slam service.
	// The typical data_frequency_hz for a lidar is 5 Hz. We're setting it here to 200 Hz
	// to speed up the tests.
	if !online {
		attrCfg.Camera = map[string]string{
			"name":              string(LidarWithErroringFunctions),
			"data_frequency_hz": "0",
		}
	} else {
		attrCfg.Camera = map[string]string{
			"name":              string(LidarWithErroringFunctions),
			"data_frequency_hz": "5",
		}
	}

	// Add movement sensor component to config (optional)
	movementSensorDone := make(chan struct{})
	if useIMU || useOdometer {
		// We're using MovementSensorWithErroringFunctions as a placeholder for deps.
		// We're defining and using the injection movement sensor
		// to overwrite this movement sensor when we create the slam service.
		// The typical data_frequency_hz for a movement sensor is 5 Hz. We're setting it
		// here to 200 Hz to speed up the tests.
		if !online {
			attrCfg.MovementSensor = map[string]string{
				"name":              string(MovementSensorWithErroringFunctions),
				"data_frequency_hz": "0",
			}
		} else {
			attrCfg.MovementSensor = map[string]string{
				"name":              string(MovementSensorWithErroringFunctions),
				"data_frequency_hz": "20",
			}
		}
		timeTracker.movementSensorTime = timeTracker.lidarTime
	}

	// Start Sensors
	timedLidar, err := integrationTimedLidar(t, attrCfg.Camera,
		defaultLidarTimeInterval, lidarDone, &timeTracker, useIMU || useOdometer)
	test.That(t, err, test.ShouldBeNil)

	var timedMovementSensor s.TimedMovementSensor
	if useIMU || useOdometer {
		timedMovementSensor, err = integrationTimedMovementSensor(t, attrCfg.MovementSensor,
			defaultMovementSensorTimeInterval, movementSensorDone, &timeTracker,
			useIMU, useOdometer)
		test.That(t, err, test.ShouldBeNil)
	}

	// Start SLAM Service
	svc, err := CreateIntegrationSLAMService(t, attrCfg, timedLidar, timedMovementSensor, logger)
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

	if useIMU || useOdometer {
		finishedProcessingMsData := utils.SelectContextOrWaitChan(ctx, movementSensorDone)
		t.Logf("movement sensor process duration %dms (timeout = %dms)", time.Since(start).Milliseconds(), testTimeout.Milliseconds())
		test.That(t, finishedProcessingMsData, test.ShouldBeTrue)
	}
	t.Logf("sensor processes have completed, all data has been ingested")

	// Test end points and retrieve internal state
	testCartographerPosition(t, svc, useIMU, useOdometer, attrCfg.Camera["name"])
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

// integrationTimedMovementSensor returns a mock timed movement sensor.
// When the mock is called, it returns the next mock movement sensor readings,
// with the ReadingTime incremented by the sensorReadingInterval.
// The Replay sensor field of the mock readings will match the replay parameter.
// When the end of the mock movement sensor readings is reached, the done channel
// is written to once so the caller can detect when all movement sensor readings
// have been emitted from the mock. This is intended to match the same
// "end of dataset" behavior of a replay sensor.
// It is important to provide deterministic time information to cartographer to
// ensure test outputs of cartographer are deterministic.
func integrationTimedMovementSensor(
	t *testing.T,
	movementSensor map[string]string,
	sensorReadingInterval time.Duration,
	done chan struct{},
	timeTracker *timeTracker,
	useIMU bool,
	useOdometer bool,
) (s.TimedMovementSensor, error) {
	// Return nil if movement sensor is not defined
	if movementSensor["name"] == "" {
		return nil, nil
	}

	// Check that the required amount of movement sensor data is present and
	// create a mock dataset from provided mock data artifact file.
	mockDataset, err := mockMovementSensorReadingsValid(t)
	if err != nil {
		return nil, err
	}

	dataFrequencyHz, err := strconv.Atoi(movementSensor["data_frequency_hz"])
	if err != nil {
		return nil, err
	}

	var i uint64
	injectMovementSensor := &inject.TimedMovementSensor{}
	injectMovementSensor.NameFunc = func() string { return movementSensor["name"] }
	injectMovementSensor.DataFrequencyHzFunc = func() int { return dataFrequencyHz }
	injectMovementSensor.TimedMovementSensorReadingFunc = func(ctx context.Context) (s.TimedMovementSensorReadingResponse, error) {
		defer timeTracker.mu.Unlock()
		// Holds the process until for all necessary lidar data has been sent to cartographer. Is always
		// true in the first iteration. This and the manual definition of timestamps allow for consistent
		// results.
		for {
			timeTracker.mu.Lock()
			if timeTracker.movementSensorTime.Before(timeTracker.nextLidarTime) {
				time.Sleep(sensorDataIngestionWaitTime)
				break
			}
			timeTracker.mu.Unlock()
		}

		// Return the ErrEndOfDataset if all movement sensor readings have been sent to cartographer or if the
		// lidar is done.
		if int(i) >= len(mockDataset.AngVelData) || timeTracker.lidarDone {
			// Sends a signal to the integration sensor's done channel the first time end of dataset has been sent
			if !timeTracker.movementSensorDone {
				done <- struct{}{}
				timeTracker.movementSensorDone = true
			}
			return s.TimedMovementSensorReadingResponse{}, replaymovementsensor.ErrEndOfDataset
		}

		// Get next movement sensor data
		resp := createTimedMovementSensorReadingResponse(mockDataset, i, timeTracker, useIMU, useOdometer)

		// Advance the data index and update time tracker (manual timestamps occurs here)
		i++
		timeTracker.movementSensorTime = timeTracker.movementSensorTime.Add(sensorReadingInterval)

		return resp, nil
	}
	injectMovementSensor.PropertiesFunc = func() s.MovementSensorProperties {
		return s.MovementSensorProperties{
			IMUSupported:      useIMU,
			OdometerSupported: useOdometer,
		}
	}

	return injectMovementSensor, nil
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

	if err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary); err != nil {
		t.Error("TEST FAILED TimedLidarReading Mock failed to parse pcd")
		return s.TimedLidarReadingResponse{}, err
	}

	resp := s.TimedLidarReadingResponse{
		Reading:     buf.Bytes(),
		ReadingTime: timeTracker.lidarTime,
	}
	return resp, nil
}

func createTimedMovementSensorReadingResponse(data movementSensorData, i uint64,
	timeTracker *timeTracker, useIMU, useOdometer bool,
) s.TimedMovementSensorReadingResponse {
	var timedIMUResponse s.TimedIMUReadingResponse
	if useIMU {
		// linear acceleration
		linAccX := data.LinAccData[i].LinAcc.X
		linAccY := data.LinAccData[i].LinAcc.Y
		linAccZ := data.LinAccData[i].LinAcc.Z

		linAcc := r3.Vector{X: linAccX, Y: linAccY, Z: linAccZ}

		// angular velocity
		angVelX := data.AngVelData[i].AngVel.X
		angVelY := data.AngVelData[i].AngVel.Y
		angVelZ := data.AngVelData[i].AngVel.Z

		angVel := spatialmath.AngularVelocity{X: angVelX, Y: angVelY, Z: angVelZ}

		// timed imu response
		timedIMUResponse = s.TimedIMUReadingResponse{
			LinearAcceleration: linAcc,
			AngularVelocity:    angVel,
			ReadingTime:        timeTracker.movementSensorTime,
		}
	}

	var timedOdometerResponse s.TimedOdometerReadingResponse
	if useOdometer {
		// position
		latitude := data.PosData[i].Coordinate.Latitude
		longitude := data.PosData[i].Coordinate.Longitude

		position := geo.NewPoint(latitude, longitude)

		// orientation
		ox := data.OrientationData[i].Orientation.Ox
		oy := data.OrientationData[i].Orientation.Oy
		oz := data.OrientationData[i].Orientation.Oz
		theta := data.OrientationData[i].Orientation.Theta

		orientation := &spatialmath.OrientationVector{
			Theta: theta,
			OX:    ox,
			OY:    oy,
			OZ:    oz,
		}

		// timed odometer response
		timedOdometerResponse = s.TimedOdometerReadingResponse{
			Position:    position,
			Orientation: orientation,
			ReadingTime: timeTracker.movementSensorTime,
		}
	}

	resp := s.TimedMovementSensorReadingResponse{
		TimedIMUResponse:      &timedIMUResponse,
		TimedOdometerResponse: &timedOdometerResponse,
	}
	return resp
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
	if len(files) < NumPointCloudFiles {
		return errors.Errorf("expected at least %v lidar reading files for integration test", NumPointCloudFiles)
	}
	for i := 0; i < NumPointCloudFiles; i++ {
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

func mockMovementSensorReadingsValid(t *testing.T) (movementSensorData, error) {
	file, err := os.Open(artifact.MustPath(mockDataPath + "/movement_sensor/data.json"))
	if err != nil {
		t.Error("TEST FAILED TimedMovementSensorReading Mock failed to open data file")
		return movementSensorData{}, err
	}
	defer func() {
		if err := file.Close(); err != nil {
			t.Error("TEST FAILED TimedMovementSensorReading Mock failed to close data file")
		}
		test.That(t, err, test.ShouldBeNil)
	}()

	byteValue, err := io.ReadAll(file)
	if err != nil {
		t.Error("TEST FAILED TimedMovementSensorReading Mock failed to read data file")
		return movementSensorData{}, err
	}

	var data movementSensorData

	if err = json.Unmarshal(byteValue, &data); err != nil {
		t.Error("TEST FAILED TimedMovementSensorReading Mock failed to unmarshal json")
		return movementSensorData{}, err
	}

	if len(data.AngVelData) != len(data.LinAccData) &&
		len(data.AngVelData) != len(data.OrientationData) &&
		len(data.AngVelData) != len(data.PosData) {
		err = errors.New("TEST FAILED TimedMovementSensorReading movement sensor readings don't contain same number of data")
		return movementSensorData{}, err
	}

	if len(data.AngVelData) < NumMovementSensorData {
		err = errors.Errorf("expected at least %v movement sensor readings for integration test", NumMovementSensorData)
		return movementSensorData{}, err
	}
	return data, nil
}
