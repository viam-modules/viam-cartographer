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
	s "github.com/viamrobotics/viam-cartographer/sensors"
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
	if config.Online {
		return config.addLidarReadingsInOnline(ctx)
	}
	return config.addLidarReadingsInOffline(ctx)
}

// addLidarReadingsInOnline ensures the most recent lidar scan, after any corresponding IMU scans, gets processed
// by cartographer.
func (config *Config) addLidarReadingsInOnline(ctx context.Context) bool {
	// get next lidar data response
	tsr, status, err := getTimedLidarReading(ctx, config)
	if err != nil {
		return status
	}

	// add lidar data to cartographer and sleep remainder of time interval
	timeToSleep := config.tryAddLidarReading(ctx, tsr)
	time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	config.Logger.Debugf("lidar sleep for %vms", timeToSleep)

	return false
}

// addLidarReadingsInOffline ensures lidar scans get added in a time ordered series with any desired
// IMU scans without skipping any.
func (config *Config) addLidarReadingsInOffline(ctx context.Context) bool {
	// Extract current IMU reading time for ordering data ingestion
	config.Mutex.Lock()
	currentIMUTime := time.Time{}
	if config.currentIMUData != nil {
		currentIMUTime = config.currentIMUData.ReadingTime
	}
	config.Mutex.Unlock()

	// If an IMU exists, skip adding measurement until the current lidar time is after the current IMU timestamp
	if config.IMU != nil && config.currentLidarData != nil && config.currentLidarData.ReadingTime.Sub(currentIMUTime).Milliseconds() > 0 {
		time.Sleep(10 * time.Millisecond)
		return false
	}

	// Add current lidar data if it is non-nil
	if config.currentLidarData != nil {
		config.tryAddLidarReadingUntilSuccess(ctx, *config.currentLidarData)

		config.Mutex.Lock()
		if config.sensorProcessStartTime == defaultTime {
			config.sensorProcessStartTime = config.currentLidarData.ReadingTime
		}
		config.Mutex.Unlock()
	}

	// get next lidar data response
	tsr, status, err := getTimedLidarReading(ctx, config)
	if err != nil {
		return status
	}

	// update stored lidar timestamp
	config.updateMutexProtectedLidarData(tsr)

	return false
}

// tryAddLidarReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode). While add lidar
// reading fails, keep trying to add the same reading - in offline mode we want to process each reading so if we cannot
// acquire the lock we should try again.
func (config *Config) tryAddLidarReadingUntilSuccess(ctx context.Context, reading s.TimedLidarSensorReadingResponse) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddLidarReading(ctx, config.Timeout, config.Lidar.Name(), reading)
			if err == nil {
				config.Logger.Debugf("%v \t | LIDAR | Success \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
			}
			config.Logger.Debugf("%v \t | LIDAR | Failure \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
		}
	}
}

// tryAddLidarReading adds a reading to the carto facade and does not retry (online).
//
//nolint:dupl
func (config *Config) tryAddLidarReading(ctx context.Context, reading s.TimedLidarSensorReadingResponse) int {
	startTime := time.Now().UTC()

	err := config.CartoFacade.AddLidarReading(ctx, config.Timeout, config.Lidar.Name(), reading)
	if err != nil {
		config.Logger.Debugf("%v \t | LIDAR | Failure \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping lidar reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping lidar reading due to error from cartofacade", "error", err)
		}
	}
	config.Logger.Debugf("%v \t | LIDAR | Success \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(1000/config.Lidar.DataFrequencyHz()-timeElapsedMs)))
}

// getTimedLidarReading returns the next lidar reading if available along with a status denoting if the
// end of dataset has been reached.
func getTimedLidarReading(ctx context.Context, config *Config) (s.TimedLidarReadingResponse, bool, error) {
	tsr, err := config.Lidar.TimedLidarReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		// only end the sensor process if we are in offline mode
		if !config.Online {
			return tsr, strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error()), err
		}
		return tsr, false, err
	}
	return tsr, false, err
}
