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
			if jobDone := config.addIMUReadingInOnline(ctx); jobDone {
				return true
			}
		}
	}
}

// addIMUReadingInOnline ensures the most recent IMU scan, after corresponding lidar scans, gets processed by cartographer.
func (config *Config) addIMUReadingInOnline(ctx context.Context) bool {
	// get next IMU data response
	tsr, status, err := getTimedMovementSensorReading(ctx, config)
	if err != nil {
		return status
	}

	// add IMU data to cartographer and sleep remainder of time interval
	timeToSleep := config.tryAddIMUReading(ctx, *tsr.TimedIMUResponse)
	time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	config.Logger.Debugf("imu sleep for %vms", timeToSleep)

	return false
}

// tryAddIMUReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode).
// While add sensor reading fails, keep trying to add the same reading - in offline mode we want to
// process each reading so if we cannot acquire the lock we should try again.
func (config *Config) tryAddIMUReadingUntilSuccess(ctx context.Context, reading s.TimedIMUReadingResponse) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.IMU.Name(), reading)
			if err == nil {
				config.Logger.Debugf("%v \t |  IMU  | Success \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
			}
			config.Logger.Debugf("%v \t |  IMU  | Failure \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
		}
	}
}

// tryAddIMUReading adds a reading to the carto facade and does not retry (online).
//
//nolint:dupl
func (config *Config) tryAddIMUReading(ctx context.Context, reading s.TimedIMUReadingResponse) int {
	startTime := time.Now().UTC()
	if err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.IMU.Name(), reading); err != nil {
		config.Logger.Debugf("%v \t |  IMU  | Failure \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping sensor reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
		}
	}
	config.Logger.Debugf("%v \t |  IMU  | Success \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(1000/config.IMU.DataFrequencyHz()-timeElapsedMs)))
}

// getTimedMovementSensorReading returns the next movement sensor reading if available along with a status
// denoting if the end of dataset has been reached.
func getTimedMovementSensorReading(ctx context.Context, config *Config) (s.TimedMovementSensorReadingResponse, bool, error) {
	tsr, err := config.IMU.TimedMovementSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		// only end the sensor process if we are in offline mode
		if !config.IsOnline {
			return tsr, strings.Contains(err.Error(), replaymovementsensor.ErrEndOfDataset.Error()), err
		}
		return tsr, false, err
	}
	return tsr, false, err
}
