// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's mapbuilder
package sensorprocess

import (
	"bytes"
	"context"
	"errors"
	"math"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/utils/contextutils"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
)

// Config holds config needed throughout the process of adding a sensor reading to the mapbuilder.
type Config struct {
	Cartofacade      cartofacade.Interface
	Lidar            lidar.Lidar
	LidarName        string
	DataRateMs       int
	Timeout          time.Duration
	Logger           golog.Logger
	TelemetryEnabled bool
}

// Start polls the lidar to get the next sensor reading and adds it to the mapBuilder.
// stops when the context is Done.
func Start(
	ctx context.Context,
	config Config,
) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
			addSensorReading(ctx, config)
		}
	}
}

// addSensorReading adds a lidar reading to the mapbuilder.
func addSensorReading(
	parentCtx context.Context,
	config Config,
) {
	ctxWithMetadata, md := contextutils.ContextWithMetadata(parentCtx)
	readingPc, err := config.Lidar.GetData(ctxWithMetadata)
	if err != nil {
		config.Logger.Warnw("Skipping sensor reading due to error getting lidar reading", "error", err)
		return
	}
	readingTime := time.Now().UTC()

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary)
	if err != nil {
		config.Logger.Warnw("Skipping sensor reading due to error converting lidar reading to PCD", "error", err)
		return
	}

	timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
	if ok {
		readingTime, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
		if err != nil {
			config.Logger.Warnw("Skipping sensor reading due to error converting replay sensor timestamp to RFC3339Nano", "error", err)
			return
		}
		addSensorReadingFromReplaySensor(ctxWithMetadata, buf.Bytes(), readingTime, config)
	} else {
		timeToSleep := addSensorReadingFromLiveReadings(ctxWithMetadata, buf.Bytes(), readingTime, config)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	}
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
			err := config.Cartofacade.AddSensorReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
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
	err := config.Cartofacade.AddSensorReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
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
