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
	s "github.com/viamrobotics/viam-cartographer/sensors"
)

// StartIMU polls the IMU to get the next sensor reading and adds it to the cartofacade.
// Stops when the context is Done.
func (config *Config) StartIMU(ctx context.Context) bool {
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

// addIMUReading adds an IMU reading to the cartofacade, using the lidar's data rate to determine whether to run in
// offline or online mode.
func (config *Config) addIMUReading(ctx context.Context) bool {
	if config.Online {
		return config.addIMUReadingInOnline(ctx)
	}
	return config.addIMUReadingInOffline(ctx)
}

// addIMUReadingInOnline ensures the most recent IMU scan, after corresponding lidar scans, gets processed by cartographer.
func (config *Config) addIMUReadingInOnline(ctx context.Context) bool {
	// get next IMU data response
	tsr, status, err := getTimedIMUSensorReading(ctx, config)
	if err != nil {
		return status
	}

	// parse IMU reading
	sr := cartofacade.IMUReading{
		LinearAcceleration: tsr.TimedIMUResponse.LinearAcceleration,
		AngularVelocity:    tsr.TimedIMUResponse.AngularVelocity,
	}

	// add IMU data to cartographer and sleep remainder of time interval
	timeToSleep := config.tryAddIMUReading(ctx, sr, tsr.TimedIMUResponse.ReadingTime)
	time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	config.Logger.Debugf("imu sleep for %vms", timeToSleep)

	return false
}

// addIMUReadingInOffline ensures IMU scans get added in a time ordered series with any desired
// lidar scans without skipping any.
func (config *Config) addIMUReadingInOffline(ctx context.Context) bool {
	// extract current lidar reading time for ordering data ingestion
	config.Mutex.Lock()
	sensorProcessStartTime := config.sensorProcessStartTime
	config.Mutex.Unlock()

	if sensorProcessStartTime != defaultTime && config.currentIMUData.time != defaultTime {
		// skip adding measurement if IMU data has been defined but occurs before first lidar data
		if sensorProcessStartTime.Sub(config.currentIMUData.time).Milliseconds() >= 0 {
			time.Sleep(10 * time.Millisecond)
			return false
		}

		// add IMU data
		config.tryAddIMUReadingUntilSuccess(ctx, config.currentIMUData.data, config.currentIMUData.time)
	}

	// get next IMU data response
	tsr, status, err := getTimedIMUSensorReading(ctx, config)
	if err != nil {
		return status
	}

	// parse IMU reading
	sr := cartofacade.IMUReading{
		LinearAcceleration: tsr.TimedIMUResponse.LinearAcceleration,
		AngularVelocity:    tsr.TimedIMUResponse.AngularVelocity,
	}

	// TODO: Remove dropping out of order IMU readings after DATA-1812 has been complete
	// JIRA Ticket: https://viam.atlassian.net/browse/DATA-1812
	// update current IMU data and time
	if config.currentIMUData.time.Sub(tsr.TimedIMUResponse.ReadingTime).Milliseconds() < 0 {
		config.updateMutexProtectedIMUData(tsr.TimedIMUResponse.ReadingTime, sr)
	} else {
		config.Logger.Debugf("%v \t | IMU | Dropping data \t \t | %v \n",
			tsr.TimedIMUResponse.ReadingTime, tsr.TimedIMUResponse.ReadingTime.Unix())
	}

	return false
}

// tryAddIMUReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode).
// While add sensor reading fails, keep trying to add the same reading - in offline mode we want to
// process each reading so if we cannot acquire the lock we should try again.
func (config *Config) tryAddIMUReadingUntilSuccess(ctx context.Context, reading cartofacade.IMUReading, readingTime time.Time) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.IMU.Name(), reading, readingTime)
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
func (config *Config) tryAddIMUReading(ctx context.Context, reading cartofacade.IMUReading, readingTime time.Time) int {
	startTime := time.Now().UTC()
	err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.IMU.Name(), reading, readingTime)
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
	return int(math.Max(0, float64(1000/config.IMU.DataFrequencyHz()-timeElapsedMs)))
}

// getTimedIMUSensorReading returns the next IMU reading if available along with a status denoting if the
// end of dataset has been reached.
func getTimedIMUSensorReading(ctx context.Context, config *Config) (s.TimedMovementSensorReadingResponse, bool, error) {
	tsr, err := config.IMU.TimedMovementSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		// only end the sensor process if we are in offline mode
		if !config.Online {
			return tsr, strings.Contains(err.Error(), replaymovementsensor.ErrEndOfDataset.Error()), err
		}
		return tsr, false, err
	}
	return tsr, false, err
}
