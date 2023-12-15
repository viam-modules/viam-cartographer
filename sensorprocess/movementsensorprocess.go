// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"math"
	"time"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	replaymovementsensor "go.viam.com/rdk/components/movementsensor/replay"
)

// StartIMU polls the IMU to get the next sensor reading and adds it to the cartofacade.
// Stops when the context is Done.
func (config *Config) StartIMU(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
			if err := config.addIMUReadingInOnline(ctx); err != nil {
				config.Logger.Warn(err)
			}
		}
	}
}

// addIMUReadingInOnline ensures the most recent IMU scan, after corresponding lidar scans, gets processed by cartographer.
func (config *Config) addIMUReadingInOnline(ctx context.Context) error {
	// get next IMU data response; ignoring status since it is always false
	imuReading, err := config.IMU.TimedMovementSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		if errors.Is(err, replaymovementsensor.ErrEndOfDataset) {
			time.Sleep(1 * time.Second)
		}
		return err
	}

	// add IMU data to cartographer and sleep remainder of time interval
	timeToSleep := config.tryAddIMUReadingOnce(ctx, *imuReading.TimedIMUResponse)

	if !imuReading.TestIsReplaySensor {
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
		config.Logger.Debugf("imu sleep for %vms", timeToSleep)
	}

	return nil
}

// tryAddIMUReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode).
// While add sensor reading fails, keep trying to add the same reading - in offline mode we want to
// process each reading so if we cannot acquire the lock we should try again.
func (config *Config) tryAddIMUReadingUntilSuccess(ctx context.Context, reading s.TimedIMUReadingResponse) error {
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
			if err := config.tryAddIMUReading(ctx, reading); err != nil {
				if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
					config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
				}
			} else {
				return nil
			}
		}
	}
}

// tryAddIMUReadingOnce adds a reading to the carto facade and does not retry. Returns remainder of time interval.
func (config *Config) tryAddIMUReadingOnce(ctx context.Context, reading s.TimedIMUReadingResponse) int {
	startTime := time.Now().UTC()
	if err := config.tryAddIMUReading(ctx, reading); err != nil {
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping sensor reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
		}
	}
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(1000/config.IMU.DataFrequencyHz()-timeElapsedMs)))
}

// tryAddIMUReading tries to add a reading to the carto facade.
func (config *Config) tryAddIMUReading(ctx context.Context, reading s.TimedIMUReadingResponse) error {
	err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.IMU.Name(), reading)
	if err != nil {
		config.Logger.Debugf("%v \t |  IMU  | Failure \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	} else {
		config.Logger.Debugf("%v \t |  IMU  | Success \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	}
	return err
}
