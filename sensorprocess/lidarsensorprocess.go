// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"math"
	"strings"
	"time"

	"go.viam.com/rdk/components/camera/replaypcd"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors"
)

// StartLidar polls the lidar to get the next sensor reading and adds it to the cartofacade.
// Stops when the context is Done.
func (config *Config) StartLidar(ctx context.Context) bool {
	for {
		select {
		case <-ctx.Done():
			return false
		default:
			if jobDone := config.addLidarReading(ctx); jobDone {
				config.Logger.Info("Beginning final optimization")
				err := config.RunFinalOptimizationFunc(ctx, config.InternalTimeout)
				if err != nil {
					config.Logger.Error("Failed to finish processing all sensor readings: ", err)
				}
				return true
			}
		}
	}
}

// addLidarReading adds a lidar reading to the cartofacade, using the lidar's data rate to determine whether to run in
// offline or online mode.
func (config *Config) addLidarReading(ctx context.Context) bool {
	if config.LidarDataFrequencyHz != 0 {
		return config.addLidarReadingsInOnline(ctx)
	}
	return config.addLidarReadingsInOffline(ctx)
}

// addLidarReadingsInOnline ensures the most recent lidar scan, after any corresponding IMU scans, gets processed
// by cartographer.
func (config *Config) addLidarReadingsInOnline(ctx context.Context) bool {
	// get next lidar data response
	tsr, status, err := getTimedLidarSensorReading(ctx, config)
	if err != nil {
		return status
	}

	// update stored lidar timestamp
	config.updateMutexProtectedLidarData(tsr.ReadingTime, tsr.Reading)

	// add lidar data to cartographer and sleep remainder of time interval
	timeToSleep := config.tryAddLidarReading(ctx, tsr.Reading, tsr.ReadingTime)
	time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	config.Logger.Debugf("lidar sleep for %vms", timeToSleep)

	return false
}

// addLidarReadingsInOffline ensures lidar scans get added in a time ordered series with any desired
// IMU scans without skipping any.
func (config *Config) addLidarReadingsInOffline(ctx context.Context) bool {
	// Extract current IMU reading time for ordering data ingestion
	config.Mutex.Lock()
	currentIMUTime := config.currentIMUData.time
	config.Mutex.Unlock()

	// If an IMU exists, skip adding measurement until the current lidar time is after the current IMU timestamp
	if config.IMUName != "" && config.currentLidarData.time.Sub(currentIMUTime).Milliseconds() > 0 {
		time.Sleep(10 * time.Millisecond)
		return false
	}

	// Add current lidar data if it is non-nil
	if config.currentLidarData.data != nil {
		config.tryAddLidarReadingUntilSuccess(ctx, config.currentLidarData.data, config.currentLidarData.time)

		config.Mutex.Lock()
		if config.sensorProcessStartTime == defaultTime {
			config.sensorProcessStartTime = config.currentLidarData.time
		}
		config.Mutex.Unlock()
	}

	// get next lidar data response
	tsr, status, err := getTimedLidarSensorReading(ctx, config)
	if err != nil {
		return status
	}

	// update stored lidar timestamp
	config.updateMutexProtectedLidarData(tsr.ReadingTime, tsr.Reading)

	return false
}

// tryAddLidarReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode). While add lidar
// reading fails, keep trying to add the same reading - in offline mode we want to process each reading so if we cannot
// acquire the lock we should try again.
func (config *Config) tryAddLidarReadingUntilSuccess(ctx context.Context, reading []byte, readingTime time.Time) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddLidarReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
			if err == nil {
				config.Logger.Debugf("%v \t | LIDAR | Success \t \t | %v \n", readingTime, readingTime.Unix())
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
			}
			config.Logger.Debugf("%v \t | LIDAR | Failure \t \t | %v \n", readingTime, readingTime.Unix())
		}
	}
}

// tryAddLidarReading adds a reading to the carto facade and does not retry (online).
func (config *Config) tryAddLidarReading(ctx context.Context, reading []byte, readingTime time.Time) int {
	startTime := time.Now().UTC()

	err := config.CartoFacade.AddLidarReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
	if err != nil {
		config.Logger.Debugf("%v \t | LIDAR | Failure \t \t | %v \n", readingTime, readingTime.Unix())
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping lidar reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping lidar reading due to error from cartofacade", "error", err)
		}
	}
	config.Logger.Debugf("%v \t | LIDAR | Success \t \t | %v \n", readingTime, readingTime.Unix())
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(1000/config.LidarDataFrequencyHz-timeElapsedMs)))
}

// getTimedLidarSensorReading returns the next lidar reading if available along with a status denoting if the
// end of dataset has been reached.
func getTimedLidarSensorReading(ctx context.Context, config *Config) (sensors.TimedLidarSensorReadingResponse, bool, error) {
	tsr, err := config.Lidar.TimedLidarSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		// only end the sensor process if we are in offline mode
		if config.LidarDataFrequencyHz == 0 {
			return tsr, strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error()), err
		}
		return tsr, false, err
	}
	return tsr, false, err
}
