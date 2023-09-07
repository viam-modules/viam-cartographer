// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"math"
	"strings"
	"time"

	replaymovementsensor "go.viam.com/rdk/components/movementsensor/replay"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors"
)

// StartIMU polls the IMU to get the next sensor reading and adds it to the cartofacade.
// stops when the context is Done.
func (config *Config) StartIMU(
	ctx context.Context,
) bool {
	for {
		select {
		case <-ctx.Done():
			return false
		default:
			if jobDone := config.addIMUReading(ctx); jobDone {
				return true
			}
		}
	}
}

// addIMUReading adds an IMU reading to the cartofacade.
func (config *Config) addIMUReading(
	ctx context.Context,
) bool {
	// Process data in online or offline mode determined by the lidar's data rate
	if config.LidarDataRateMsec != 0 {
		/*
			when the lidar data rate msec is non-zero, we assume the user wants to be in "online"
			mode and ensure the most recent scan gets processed by cartographer.
		*/
		return config.addIMUReadingInOnline(ctx)
	} else {
		/*
			In order for cartographer to build a correct map, the lidar and imu readings need to be processed in
			order in offline mode. We only add the stored IMU data if the next lidar data has a timestamp after
			the current IMU reading's timestamp.
		*/
		return config.addIMUReadingInOffline(ctx)
	}
}

func (config *Config) addIMUReadingInOnline(ctx context.Context) bool {
	// get next imu data response
	tsr, status, err := getTimedIMUSensorReading(ctx, config)
	if err != nil {
		return status
	}

	// parse imu reading
	sr := cartofacade.IMUReading{
		LinearAcceleration: tsr.LinearAcceleration,
		AngularVelocity:    tsr.AngularVelocity,
	}

	// update stored imu time
	config.updateMutexProtectedIMUData(tsr.ReadingTime, sr)

	config.Mutex.Lock()
	currentLidarData := config.currentLidarData.time
	cfg := *config
	config.Mutex.Unlock()

	// only add imu data to cartographer and sleep remainder of time interval if it is after most recent lidar data to ensure
	// ordered time
	if tsr.ReadingTime.Sub(currentLidarData).Milliseconds() > 0 {
		timeToSleep := tryAddIMUReading(ctx, sr, tsr.ReadingTime, cfg)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	} else {
		config.Logger.Debugf("%v \t |  IMU  | Failure \t \t | %v \n", tsr.ReadingTime, tsr.ReadingTime.Unix())
	}
	return false
}

func (config *Config) addIMUReadingInOffline(ctx context.Context) bool {
	config.Mutex.Lock()
	currentIMUTime := config.currentIMUData.time
	currentIMUData := config.currentIMUData.data
	firstLidarReadingTime := config.firstLidarReadingTime
	cfg := *config
	config.Mutex.Unlock()

	if firstLidarReadingTime != defaultTime && currentIMUTime != defaultTime {
		// Skip adding measurement if imu data has been defined but occurs before first lidar data
		if firstLidarReadingTime.Sub(currentIMUTime).Milliseconds() >= 0 {
			time.Sleep(10 * time.Millisecond)
			return false
		}

		// Add IMU data
		tryAddIMUReadingUntilSuccess(ctx, currentIMUData, currentIMUTime, cfg)
	}

	// get next imu data response
	tsr, status, err := getTimedIMUSensorReading(ctx, config)
	if err != nil {
		return status
	}

	// parse imu reading
	sr := cartofacade.IMUReading{
		LinearAcceleration: tsr.LinearAcceleration,
		AngularVelocity:    tsr.AngularVelocity,
	}

	// TODO: Remove dropping out of order imu readings after DATA-1812 has been complete
	// JIRA Ticket: https://viam.atlassian.net/browse/DATA-1812
	// update current imu data and time
	if currentIMUTime.Sub(tsr.ReadingTime).Milliseconds() < 0 {
		config.updateMutexProtectedIMUData(tsr.ReadingTime, sr)
	} else {
		config.Logger.Debugf("%v \t | IMU | Dropping data \t \t | %v \n", tsr.ReadingTime, tsr.ReadingTime.Unix())
	}

	return false
}

// tryAddIMUReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode).
func tryAddIMUReadingUntilSuccess(ctx context.Context, reading cartofacade.IMUReading, readingTime time.Time, config Config) {
	/*
		while add sensor reading fails, keep trying to add the same reading - in offline mode
		we want to process each reading so if we cannot acquire the lock we should try again
	*/
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.IMUName, reading, readingTime)
			if err == nil {
				config.Logger.Debugf("%v \t |  IMU  | Success \t \t | %v \n", readingTime, readingTime.Unix())
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
			}
			config.Logger.Debugf("%v \t |  IMU  | Failure \t \t | %v \n", readingTime, readingTime.Unix())
		}
	}
}

// tryAddIMUReading adds a reading to the carto facade and does not retry (online).
func tryAddIMUReading(ctx context.Context, reading cartofacade.IMUReading, readingTime time.Time, config Config) int {
	startTime := time.Now().UTC()
	err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.IMUName, reading, readingTime)
	if err != nil {
		config.Logger.Debugf("%v \t |  IMU  | Failure \t \t | %v \n", readingTime, readingTime.Unix())
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping sensor reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
		}
	}
	config.Logger.Debugf("%v \t |  IMU  | Success \t \t | %v \n", readingTime, readingTime.Unix())
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(config.IMUDataRateMsec-timeElapsedMs)))
}

// getTimedIMUSensorReading returns the next imu reading if available along with a status denoting if the end of dataset has been reached.
func getTimedIMUSensorReading(ctx context.Context, config *Config) (sensors.TimedIMUSensorReadingResponse, bool, error) {
	tsr, err := config.IMU.TimedIMUSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		// only end the sensor process if we are in offline mode
		if config.LidarDataRateMsec == 0 {
			return tsr, strings.Contains(err.Error(), replaymovementsensor.ErrEndOfDataset.Error()), err
		}
		return tsr, false, err
	}
	return tsr, false, err
}
