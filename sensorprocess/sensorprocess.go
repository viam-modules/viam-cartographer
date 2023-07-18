// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"math"
	"strings"
	"time"

	"github.com/edaniels/golog"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors"
)

// Config holds config needed throughout the process of adding a sensor reading to the cartofacade.
type Config struct {
	CartoFacade      cartofacade.Interface
	Lidar            sensors.TimedSensor
	LidarName        string
	DataRateMs       int
	Timeout          time.Duration
	Logger           golog.Logger
	TelemetryEnabled bool
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
			if jobDone := addSensorReading(ctx, config); jobDone {
				return true
			}
		}
	}
}

// addSensorReading adds a lidar reading to the cartofacade.
func addSensorReading(
	ctx context.Context,
	config Config,
) bool {
	tsr, err := config.Lidar.TimedSensorReading(ctx)
	if err != nil {
		if strings.Contains(err.Error(), "reached end of dataset") {
			config.Logger.Warn(err)
			return true
		}
		config.Logger.Warn(err)
		return false
	}

	if tsr.Replay {
		addSensorReadingFromReplaySensor(ctx, tsr.Reading, tsr.ReadingTime, config)
	} else {
		timeToSleep := addSensorReadingFromLiveReadings(ctx, tsr.Reading, tsr.ReadingTime, config)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	}
	return false
}

// addSensorReadingFromReplaySensor adds a reading from a replay sensor to the cartofacade
// retries on error.
func addSensorReadingFromReplaySensor(ctx context.Context, reading []byte, readingTime time.Time, config Config) {
	/*
		while add sensor reading fails, keep trying to add the same reading - in offline mode
		we want to process each reading so if we cannot acquire the lock we should try again
	*/
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddSensorReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
			if err == nil {
				// TODO: increment telemetry counter success
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				// TODO: increment telemetry counter unexpected error
				config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
			}
			// TODO: increment telemetry counter unable to acquire lock
		}
	}
}

// addSensorReadingFromLiveReadings adds a reading from a live lidar to the carto facade
// does not retry.
func addSensorReadingFromLiveReadings(ctx context.Context, reading []byte, readingTime time.Time, config Config) int {
	startTime := time.Now()
	err := config.CartoFacade.AddSensorReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
	if err != nil {
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping sensor reading due to lock contention in cartofacade", "error", err)
			// TODO: increment telemetry counter unable to acquire lock
		} else {
			config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
			// TODO: increment telemetry counter unexpected error
		}
	}
	// TODO: increment telemetry counter success
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(config.DataRateMs-timeElapsedMs)))
}
