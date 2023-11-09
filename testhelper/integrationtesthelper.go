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
	replaylidar "go.viam.com/rdk/components/camera/replaypcd"
	replaymovementsensor "go.viam.com/rdk/components/movementsensor/replay"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/artifact"

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
	mockDataPath                = "viam-cartographer/mock_data"
	sensorDataIngestionWaitTime = 50 * time.Millisecond
)

var defaultTime = time.Time{}

// TimeTracker stores the current and next timestamps for both IMU and lidar. These are used to manually
// set the timestamp of each set of data being sent to cartographer and ensure proper ordering between them.
// This allows for consistent testing.
type TimeTracker struct {
	LidarTime     time.Time
	NextLidarTime time.Time

	ImuTime     time.Time
	NextImuTime time.Time

	LastLidarTime time.Time
	LastImuTime   time.Time

	Mu *sync.Mutex
}

// IntegrationTimedLidar returns a mock timed lidar sensor
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
func IntegrationTimedLidar(
	t *testing.T,
	lidarName string,
	replay bool,
	sensorReadingInterval time.Duration,
	done chan struct{},
	timeTracker *TimeTracker,
) (s.TimedLidar, error) {
	// Check that the required amount of lidar data is present
	err := mockLidarReadingsValid()
	if err != nil {
		return nil, err
	}

	var i uint64

	closed := false
	injectLidar := &inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return lidarName }
	injectLidar.TimedLidarReadingFunc = func(ctx context.Context) (s.TimedLidarReadingResponse, error) {
		defer timeTracker.Mu.Unlock()
		/*
			Holds the process until for all necessary IMU data has been sent to cartographer. Only applicable
			when the IMU is present (timeTracker.NextImuTime has been defined) and is always true in the first iteration.
			This and the manual definition of timestamps allow for consistent results.
		*/
		for {
			timeTracker.Mu.Lock()
			if timeTracker.ImuTime == defaultTime {
				time.Sleep(sensorDataIngestionWaitTime)
				break
			}

			if i <= 1 || timeTracker.LidarTime.Sub(timeTracker.ImuTime) <= 0 {
				time.Sleep(sensorDataIngestionWaitTime)
				break
			}
			timeTracker.Mu.Unlock()
		}

		// Communicate that all lidar readings have been sent to cartographer or if the last IMU reading has been sent,
		// checks if LastLidarTime has been defined. If so, simulate endOfDataSet error.
		t.Logf("TimedLidarReading Mock i: %d, closed: %v, readingTime: %s\n", i, closed, timeTracker.LidarTime.String())
		if i >= NumPointClouds || timeTracker.LastImuTime != defaultTime {
			// Sends a signal to the integration sensor's done channel the first time end of dataset has been sent
			if !closed {
				done <- struct{}{}
				closed = true
				timeTracker.LastLidarTime = timeTracker.LidarTime
			}

			return s.TimedLidarReadingResponse{}, replaylidar.ErrEndOfDataset
		}

		// Get next lidar data
		resp, err := createTimedLidarReadingResponse(t, i, replay, timeTracker)
		if err != nil {
			return resp, err
		}

		// Advance the data index and update time tracker (manual timestamps occurs here)
		i++
		timeTracker.LidarTime = timeTracker.LidarTime.Add(sensorReadingInterval)
		timeTracker.NextLidarTime = timeTracker.LidarTime.Add(sensorReadingInterval)

		return resp, nil
	}

	return injectLidar, nil
}

// IntegrationTimedIMU returns a mock timed IMU sensor.
// When the mock is called, it returns the next mock IMU readings, with the
// ReadingTime incremented by the sensorReadingInterval.
// The Replay sensor field of the mock readings will match the replay parameter.
// When the end of the mock IMU readings is reached, the done channel
// is written to once so the caller can detect when all IMU readings have been emitted
// from the mock. This is intended to match the same "end of dataset" behavior of a
// replay sensor.
// It is important to provide deterministic time information to cartographer to
// ensure test outputs of cartographer are deterministic.
func IntegrationTimedIMU(
	t *testing.T,
	movementSensorName string,
	replay bool,
	sensorReadingInterval time.Duration,
	done chan struct{},
	timeTracker *TimeTracker,
) (s.TimedMovementSensor, error) {
	// Return nil if IMU is not requested
	if movementSensorName == "" {
		return nil, nil
	}

	// Check that the required amount of IMU data is present and create a mock dataset from provided mock data artifact file.
	mockDataset, err := mockIMUReadingsValid(t)
	if err != nil {
		return nil, err
	}

	var i uint64
	closed := false
	injectIMU := &inject.TimedMovementSensor{}
	injectIMU.NameFunc = func() string { return imuName }
	injectIMU.TimedMovementSensorReadingFunc = func(ctx context.Context) (s.TimedMovementSensorReadingResponse, error) {
		defer timeTracker.Mu.Unlock()
		/*
			Holds the process until for all necessary lidar data has been sent to cartographer. Is always
			true in the first iteration. This and the manual definition of timestamps allow for consistent
			results.
		*/
		for {
			timeTracker.Mu.Lock()
			if i == 0 || timeTracker.ImuTime.Sub(timeTracker.NextLidarTime) < 0 {
				time.Sleep(sensorDataIngestionWaitTime)
				break
			}
			timeTracker.Mu.Unlock()
		}

		// Communicate that all desired IMU readings have been sent or to cartographer or if the last lidar reading
		// has been sent by, checks if LastLidarTime has been defined. If so, simulate endOfDataSet error.
		t.Logf("TimedMovementSensorReading Mock i: %d, closed: %v, readingTime: %s\n", i, closed, timeTracker.ImuTime.String())
		if int(i) >= len(mockDataset) || timeTracker.LastLidarTime != defaultTime {
			// Sends a signal to the integration sensor's done channel the first time end of dataset has been sent
			if !closed {
				done <- struct{}{}
				closed = true
				timeTracker.LastImuTime = timeTracker.ImuTime
			}
			return s.TimedMovementSensorReadingResponse{}, replaymovementsensor.ErrEndOfDataset
		}

		// Get next IMU data
		resp, err := createTimedMovementSensorReadingResponse(t, mockDataset[i], replay, timeTracker)
		if err != nil {
			return resp, err
		}

		// Advance the data index and update time tracker (manual timestamps occurs here)
		i++
		timeTracker.ImuTime = timeTracker.ImuTime.Add(sensorReadingInterval)
		timeTracker.NextImuTime = timeTracker.ImuTime.Add(sensorReadingInterval)

		return resp, nil
	}
	injectIMU.PropertiesFunc = func() s.MovementSensorProperties {
		return s.MovementSensorProperties{
			IMUSupported: true,
		}
	}

	return injectIMU, nil
}

func createTimedLidarReadingResponse(t *testing.T, i uint64, replay bool, timeTracker *TimeTracker,
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
		Reading:        buf.Bytes(),
		ReadingTime:    timeTracker.LidarTime,
		IsReplaySensor: replay,
	}
	return resp, nil
}

func createTimedMovementSensorReadingResponse(t *testing.T, line string, replay bool, timeTracker *TimeTracker,
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
			ReadingTime:        timeTracker.ImuTime,
		},
		IsReplaySensor: replay,
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
