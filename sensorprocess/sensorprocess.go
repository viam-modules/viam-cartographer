// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"math"
	"strings"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/components/camera/replaypcd"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors"
)

// Config holds config needed throughout the process of adding a sensor reading to the cartofacade.
type Config struct {
	CartoFacade       cartofacade.Interface
	Lidar             sensors.TimedLidarSensor
	LidarName         string
	LidarDataRateMsec int
	Timeout           time.Duration
	Logger            golog.Logger
}

// Start polls the lidar to get the next sensor reading and adds it to the cartofacade.
// stops when the context is Done.
func Start(
	ctx context.Context,
	config Config,
) bool {
	for {
		select {
		case <-ctx.Done():
			return false
		default:
			if jobDone := addLidarReading(ctx, config); jobDone {
				return true
			}
		}
	}
}

// addLidarReading adds a lidar reading to the cartofacade.
func addLidarReading(
	ctx context.Context,
	config Config,
) bool {
	tsr, err := config.Lidar.TimedLidarSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		// only end the sensor process if we are in offline mode
		if config.LidarDataRateMsec == 0 {
			return strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error())
		}
		return false
	}
	/*
	 when the data rate msec is 0, we assume the user wants to be in "offline"
	 mode and ensure every scan gets processed by cartographer
	*/
	if config.LidarDataRateMsec == 0 {
		tryAddLidarReadingUntilSuccess(ctx, tsr.Reading, tsr.ReadingTime, config)
	} else {
		timeToSleep := tryAddLidarReading(ctx, tsr.Reading, tsr.ReadingTime, config)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
		config.Logger.Debugf("sleep for %s milliseconds", time.Duration(timeToSleep))
	}
	return false
}

// tryAddLidarReadingUntilSuccess adds a reading to the cartofacade
// retries on error (offline mode).
func tryAddLidarReadingUntilSuccess(ctx context.Context, reading []byte, readingTime time.Time, config Config) {
	/*
		while add sensor reading fails, keep trying to add the same reading - in offline mode
		we want to process each reading so if we cannot acquire the lock we should try again
	*/
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddLidarReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
			if err == nil {
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
			}
		}
	}
}

// tryAddLidarReading adds a reading to the carto facade
// does not retry (online).
func tryAddLidarReading(ctx context.Context, reading []byte, readingTime time.Time, config Config) int {
	startTime := time.Now()
	err := config.CartoFacade.AddLidarReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
	if err != nil {
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping sensor reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
		}
	}
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(config.LidarDataRateMsec-timeElapsedMs)))
}
