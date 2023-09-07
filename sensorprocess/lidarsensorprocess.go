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
// stops when the context is Done.
func (config *Config) StartLidar(
	ctx context.Context,
) bool {
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

// addLidarReading adds a lidar reading to the cartofacade.
func (config *Config) addLidarReading(ctx context.Context) bool {
	// Process data in online or offline mode determined by the lidar's data rate
	if config.LidarDataRateMsec != 0 {
		/*
			when the lidar data rate msec is non-zero, we assume the user wants to be in "online"
			mode and ensure the most recent scan after any corresponding imu scans gets processed by cartographer.
		*/
		return config.addLidarReadingsInOnline(ctx)
	} else {
		/*
			In order for cartographer to build a correct map, the lidar and imu readings need to be processed in
			order in offline mode. We only add the stored lidar data if we do not have any IMU data to add, or if
			the next IMU data has a timestamp after the current lidar reading's timestamp.
		*/
		return config.addLidarReadingsInOffline(ctx)
	}
}

func (config *Config) addLidarReadingsInOnline(ctx context.Context) bool {
	// get next lidar data response
	tsr, status, err := getTimedLidarSensorReading(ctx, config)
	if err != nil {
		return status
	}

	// update stored lidar timestamp and extract current timestamps
	config.updateMutexProtectedLidarData(tsr.ReadingTime, tsr.Reading)

	config.Mutex.Lock()
	currentIMUTime := config.currentIMUData.time
	cfg := *config
	config.Mutex.Unlock()

	// if an imu exists only add lidar data to cartographer and sleep remainder of time interval if it is after most recent imu data
	// to ensure ordered time
	if config.IMUName == "" || tsr.ReadingTime.Sub(currentIMUTime) >= 0 {
		timeToSleep := tryAddLidarReading(ctx, tsr.Reading, tsr.ReadingTime, cfg)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
		config.Logger.Debugf("sleep for %vms", timeToSleep)
	} else {
		config.Logger.Debugf("%v \t | LIDAR | Failure \t \t | %v \n", tsr.ReadingTime, tsr.ReadingTime.Unix())
	}

	return false
}

func (config *Config) addLidarReadingsInOffline(ctx context.Context) bool {

	config.Mutex.Lock()
	currentLidarTime := config.currentLidarData.time
	currentLidarData := config.currentLidarData.data
	currentIMUTime := config.currentIMUData.time
	cfg := *config
	config.Mutex.Unlock()

	// If an IMU exists, skip adding measurement until the current lidar time is after the current imu timestamp
	if config.IMUName != "" && currentLidarTime.Sub(currentIMUTime).Milliseconds() > 0 {
		time.Sleep(10 * time.Millisecond)
		return false
	}

	// Add current lidar data if it is non-nil
	if currentLidarData != nil {
		tryAddLidarReadingUntilSuccess(ctx, currentLidarData, currentLidarTime, cfg)

		config.Mutex.Lock()
		if config.firstLidarReadingTime == defaultTime {
			config.firstLidarReadingTime = currentLidarTime
		}
		config.Mutex.Unlock()
	}

	// get next lidar data response
	tsr, status, err := getTimedLidarSensorReading(ctx, config)
	if err != nil {
		return status
	}

	config.updateMutexProtectedLidarData(tsr.ReadingTime, tsr.Reading)

	return false
}

// tryAddLidarReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode).
func tryAddLidarReadingUntilSuccess(ctx context.Context, reading []byte, readingTime time.Time, config Config) {
	/*
		while add lidar reading fails, keep trying to add the same reading - in offline mode
		we want to process each reading so if we cannot acquire the lock we should try again
	*/
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
func tryAddLidarReading(ctx context.Context, reading []byte, readingTime time.Time, config Config) int {
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
	return int(math.Max(0, float64(config.LidarDataRateMsec-timeElapsedMs)))
}

// getTimedLidarSensorReading returns the next lidar reading if available along with a status denoting if the end of dataset has been
// reached.
func getTimedLidarSensorReading(ctx context.Context, config *Config) (sensors.TimedLidarSensorReadingResponse, bool, error) {
	tsr, err := config.Lidar.TimedLidarSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		// only end the sensor process if we are in offline mode
		if config.LidarDataRateMsec == 0 {
			return tsr, strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error()), err
		}
		return tsr, false, err
	}
	return tsr, false, err
}
