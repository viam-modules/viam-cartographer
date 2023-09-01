// Package testhelper provides test helpers which don't depend on viamcartographer
package testhelper

import (
	"bufio"
	"bytes"
	"context"
	"fmt"
	"os"
	"path"
	"regexp"
	"strconv"
	"sync"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

const (
	// NumPointClouds is the amount of mock lidar data saved in the mock_data/lidar slam artifact
	// directory used for integration tests.
	NumPointClouds = 9
	// NumIMUData is the amount of mock imu data saved in the mock_data/imu/data.txt slam artifact
	// file used for integration tests.
	NumIMUData = 30
	// Path to slam mock data used for integration tests artifact path.
	// artifact.MustPath("viam-cartographer/mock_lidar")
	mockDataPath = "/Users/jeremyhyde/Development/viam-cartographer/mock_data"
)

var defaultTime = time.Time{}

// TimeTracker stores the current and next timestamps for both imu and lidar. These are used to manually
// set the timestamp of each set of data being sent to cartographer and ensure proper ordering between them.
// This allows for consistent testing.
type TimeTracker struct {
	LidarTime     time.Time
	NextLidarTime time.Time

	ImuTime     time.Time
	NextImuTime time.Time

	LastLidarTime time.Time
	LastImuTime   time.Time

	mutex sync.Mutex
}

// IntegrationTimedLidarSensor returns a mock timed lidar sensor
// or an error if preconditions to build the mock are not met.
// It validates that all required mock lidar reading files are able to be found.
// When the mock is called, it returns the next mock lidar reading, with the
// ReadingTime incremented by the sensorReadingInterval.
// The Replay sensor field of the mock readings will match the replay parameter.
// When the end of the mock lidar readings is reached, the done channel
// is written to once so the caller can detect all lidar readings have been emitted
// from the mock. This is intended to match the same "end of dataset" behavior of a
// replay sensor.
// It is important to provide deterministic time information to cartographer to
// ensure test outputs of cartographer are deterministic.
func IntegrationTimedLidarSensor(
	t *testing.T,
	lidar string,
	replay bool,
	sensorReadingInterval time.Duration,
	done chan struct{},
	timeTracker *TimeTracker,
) (s.TimedLidarSensor, error) {

	// Check required amount of lidar data is present
	err := mockLidarReadingsValid()
	if err != nil {
		return nil, err
	}

	var i uint64
	started := false
	closed := false
	ts := &s.TimedLidarSensorMock{}
	ts.TimedLidarSensorReadingFunc = func(ctx context.Context) (s.TimedLidarSensorReadingResponse, error) {
		defer timeTracker.mutex.Unlock()

		// Holds the process until for all necessary imu data has been sent to cartographer. Only applicable
		// when the imu is present (timeTracker.NextImuTime has been defined) and is always true the first iteration.
		// This combined with manual definition of timestamps allow for consistent results.
		for {
			t.Logf("Lidar | Lidar Time: %v IMU Time %v | Next Lidar Time: %v Next IMU Time %v \n", timeTracker.LidarTime, timeTracker.ImuTime, timeTracker.NextLidarTime, timeTracker.NextImuTime)
			if !started || timeTracker.ImuTime == defaultTime || timeTracker.LidarTime.Sub(timeTracker.NextImuTime) <= 0 {
				started = true
				timeTracker.mutex.Lock()
				break
			}
			//time.Sleep(100 * time.Millisecond)
		}

		// Communicating all lidar readings have been sent to cartographer or if the last imu reading has been sent by checking
		// if LastLidarTime has been defined. If so, simulate endOfDataSet error.
		t.Logf("TimedLidarSensorReading Mock i: %d, closed: %v, readingTime: %s\n", i, closed, timeTracker.LidarTime.String())
		if i >= NumPointClouds || timeTracker.LastImuTime != defaultTime {
			// Sends a signal to the integration sensor's done channel the first time end of dataset has been sent
			if !closed {
				fmt.Println("Sending end of dataset to lidar done channel")
				done <- struct{}{}
				closed = true
				timeTracker.LastLidarTime = timeTracker.LidarTime
			}

			return s.TimedLidarSensorReadingResponse{}, errors.New("Lidar: end of dataset")
		}

		// Get next lidar data
		resp, err := createTimedLidarSensorReadingResponse(t, ctx, i, replay, timeTracker)
		if err != nil {
			return resp, nil
		}

		// Advance the data index and update time tracker (manual timestamps occurs here)
		i++
		timeTracker.LidarTime = timeTracker.LidarTime.Add(sensorReadingInterval)
		timeTracker.NextLidarTime = timeTracker.LidarTime.Add(sensorReadingInterval)

		return resp, nil
	}

	return ts, nil
}

// IntegrationTimedIMUSensor returns a mock timed IMU sensor.
// When the mock is called, it returns the next mock IMU readings, with the
// ReadingTime incremented by the sensorReadingInterval.
// The Replay sensor field of the mock readings will match the replay parameter.
// When the end of the mock IMU readings is reached, the done channel
// is written to once so the caller can detect all IMU readings have been emitted
// from the mock. This is intended to match the same "end of dataset" behavior of a
// replay sensor.
// It is important to provide deterministic time information to cartographer to
// ensure test outputs of cartographer are deterministic.
// Note that IMU replay sensors are not yet fully supported.
func IntegrationTimedIMUSensor(
	t *testing.T,
	imu string,
	replay bool,
	sensorReadingInterval time.Duration,
	done chan struct{},
	timeTracker *TimeTracker,
) (s.TimedIMUSensor, error) {

	// Return nil if imu is not requested
	if imu == "" {
		return nil, nil
	}

	// Check required amount of imu data is present and creates mock dataset from provided mock data artifact file.
	mockDataset, err := mockIMUReadingsValid(t)
	if err != nil {
		return nil, err
	}

	var i uint64
	started := false
	closed := false
	ts := &s.TimedIMUSensorMock{}
	ts.TimedIMUSensorReadingFunc = func(ctx context.Context) (s.TimedIMUSensorReadingResponse, error) {
		defer timeTracker.mutex.Unlock()

		// Holds the process until for all necessary lidar data has been sent to cartographer. Is always
		// true the first iteration. This combined with manual definition of timestamps allow for consistent
		// results.
		for {
			t.Logf("IMU   | Lidar Time: %v IMU Time %v | Next Lidar Time: %v Next IMU Time %v \n", timeTracker.LidarTime, timeTracker.ImuTime, timeTracker.NextLidarTime, timeTracker.NextImuTime)
			if !started || timeTracker.ImuTime.Sub(timeTracker.NextLidarTime) < 0 {
				started = true
				timeTracker.mutex.Lock()
				break
			}
			//time.Sleep(100 * time.Millisecond)
		}

		// Communicating all desired imu readings have been sent or to cartographer or if the last lidar reading has been sent by checking
		// if LastLidarTime has been defined. If so, simulate endOfDataSet error.
		t.Logf("TimedIMUSensorReading Mock i: %d, closed: %v, readingTime: %s\n", i, closed, timeTracker.ImuTime.String())
		if int(i) >= len(mockDataset) || timeTracker.LastLidarTime != defaultTime {
			// Sends a signal to the integration sensor's done channel the first time end of dataset has been sent
			if !closed {
				fmt.Println("Sending end of dataset to imu done channel")
				done <- struct{}{}
				closed = true
				timeTracker.LastImuTime = timeTracker.ImuTime
			}
			return s.TimedIMUSensorReadingResponse{}, errors.New("IMU: end of dataset")
		}

		resp, err := createTimedIMUSensorReadingResponse(t, ctx, mockDataset[i], replay, timeTracker)
		if err != nil {
			return resp, err
		}

		i++
		timeTracker.ImuTime = timeTracker.ImuTime.Add(sensorReadingInterval)
		timeTracker.NextImuTime = timeTracker.ImuTime.Add(sensorReadingInterval)

		return resp, nil
	}

	return ts, nil
}

func createDatasetFromFile(t *testing.T, filepath string) ([]string, error) {
	file, err := os.Open(mockDataPath + "/imu/data.txt")
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
	return mockDataset, nil
}

func createTimedLidarSensorReadingResponse(t *testing.T, ctx context.Context, i uint64, replay bool, timeTracker *TimeTracker,
) (s.TimedLidarSensorReadingResponse, error) {
	file, err := os.Open(mockDataPath + "/lidar/" + strconv.FormatUint(i, 10) + ".pcd")
	// file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/" + strconv.FormatUint(i, 10) + ".pcd"))
	if err != nil {
		t.Error("TEST FAILED TimedLidarSensorReading Mock failed to open pcd file")
		return s.TimedLidarSensorReadingResponse{}, err
	}
	readingPc, err := pointcloud.ReadPCD(file)
	if err != nil {
		t.Error("TEST FAILED TimedLidarSensorReading Mock failed to read pcd")
		return s.TimedLidarSensorReadingResponse{}, err
	}

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary)
	if err != nil {
		t.Error("TEST FAILED TimedLidarSensorReading Mock failed to parse pcd")
		return s.TimedLidarSensorReadingResponse{}, err
	}

	resp := s.TimedLidarSensorReadingResponse{Reading: buf.Bytes(), ReadingTime: timeTracker.LidarTime, Replay: replay}
	return resp, nil
}

func createTimedIMUSensorReadingResponse(t *testing.T, ctx context.Context, line string, replay bool, timeTracker *TimeTracker,
) (s.TimedIMUSensorReadingResponse, error) {
	re := regexp.MustCompile(`[-+]?\d*\.?\d+`)
	matches := re.FindAllString(line, -1)

	linAccX, err1 := strconv.ParseFloat(matches[0], 64)
	linAccY, err2 := strconv.ParseFloat(matches[1], 64)
	linAccZ, err3 := strconv.ParseFloat(matches[2], 64)
	if err1 != nil || err2 != nil || err3 != nil {
		t.Error("TEST FAILED TimedIMUSensorReading Mock failed to parse linear acceleration")
		return s.TimedIMUSensorReadingResponse{}, errors.New("error parsing linear acceleration from file")
	}
	linAcc := r3.Vector{X: linAccX, Y: linAccY, Z: linAccZ}

	angVelX, err1 := strconv.ParseFloat(matches[3], 64)
	angVelY, err2 := strconv.ParseFloat(matches[4], 64)
	angVelZ, err3 := strconv.ParseFloat(matches[5], 64)
	if err1 != nil || err2 != nil || err3 != nil {
		t.Error("TEST FAILED TimedIMUSensorReading Mock failed to parse angular velocity")
		return s.TimedIMUSensorReadingResponse{}, errors.New("error parsing angular velocity from file")
	}
	angVel := spatialmath.AngularVelocity{X: angVelX, Y: angVelY, Z: angVelZ}

	resp := s.TimedIMUSensorReadingResponse{LinearAcceleration: linAcc, AngularVelocity: angVel, ReadingTime: timeTracker.ImuTime, Replay: replay}
	return resp, nil
}

func mockLidarReadingsValid() error {
	dirEntries, err := os.ReadDir(mockDataPath + "/lidar") // os.ReadDir(mockDataPath)
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
	file, err := os.Open(mockDataPath + "/imu/data.txt")
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
