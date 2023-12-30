// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"math"
	"time"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
)

// StartLidar polls the lidar to get the next sensor reading and adds it to the cartofacade.
// Stops when the context is Done.
func (config *Config) StartLidar(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
			if err := config.addLidarReadingInOnline(ctx); err != nil {
				config.Logger.Warn(err)
			}
		}
	}
}

// addLidarReadingsInOnline ensures the most recent lidar scan, after any corresponding IMU scans, gets processed
// by cartographer.
func (config *Config) addLidarReadingInOnline(ctx context.Context) error {
	// get next lidar data response; ignoring status since it is always false
	lidarReading, err := config.Lidar.TimedLidarReading(ctx)
	if err != nil {
		return err
	}

	// add lidar data to cartographer and sleep remainder of time interval
	timeToSleep := config.tryAddLidarReadingOnce(ctx, lidarReading)
	time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	config.Logger.Debugf("lidar sleep for %vms", timeToSleep)
	return nil
}

// tryAddLidarReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode). While add lidar
// reading fails, keep trying to add the same reading - in offline mode we want to process each reading so if we cannot
// acquire the lock we should try again.
func (config *Config) tryAddLidarReadingUntilSuccess(ctx context.Context, reading s.TimedLidarReadingResponse) error {
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
			if err := config.tryAddLidarReading(ctx, reading); err != nil {
				if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
					config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
				}
			} else {
				return nil
			}
		}
	}
}

// tryAddLidarReadingOnce adds a reading to the carto facade and does not retry. Returns remainder of time interval.
func (config *Config) tryAddLidarReadingOnce(ctx context.Context, reading s.TimedLidarReadingResponse) int {
	startTime := time.Now().UTC()

	if err := config.tryAddLidarReading(ctx, reading); err != nil {
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping lidar reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping lidar reading due to error from cartofacade", "error", err)
		}
	}
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(1000/config.Lidar.DataFrequencyHz()-timeElapsedMs)))
}

// tryAddLidarReading tries to add a reading to the carto facade.
func (config *Config) tryAddLidarReading(ctx context.Context, reading s.TimedLidarReadingResponse) error {
	err := config.CartoFacade.AddLidarReading(ctx, config.Timeout, config.Lidar.Name(), reading)
	if err != nil {
		config.Logger.Debugf("%v \t | LIDAR | Failure \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	} else {
		config.Logger.Debugf("%v \t | LIDAR | Success \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	}
	return err
}
