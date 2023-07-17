// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/resource"
	statz "go.viam.com/utils/perf/statz"
	"go.viam.com/utils/perf/statz/units"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors"
)

// Config holds config needed throughout the process of adding a sensor reading to the cartofacade.
type Config struct {
	CartoFacade         cartofacade.Interface
	Lidar               sensors.TimedSensor
	LidarName           string
	DataRateMs          int
	Timeout             time.Duration
	Logger              golog.Logger
	TelemetryEnabled    bool
	ResourceName        resource.Name
	lidarReadingCounter *statz.Counter1[string]
}

const (
	successfulReadings = "successful readings"
	lockNotAcquired    = "unsuccessful readings - unable to acquire lock"
	unexpectedError    = "unsuccessful readings - unexpected error"
)

// Start polls the lidar to get the next sensor reading and adds it to the mapBuilder.
// stops when the context is Done.
func Start(
	ctx context.Context,
	config Config,
) {
	if config.TelemetryEnabled {
		lidarReadingCounter, err := setupTelemetry(config.ResourceName.Name)
		if err != nil {
			config.Logger.Errorf("Could not start telemetry: %s", err)
			return
		}
		config.lidarReadingCounter = lidarReadingCounter
	}

	for {
		select {
		case <-ctx.Done():
			return
		default:
			addSensorReading(ctx, config)
		}
	}
}

func setupTelemetry(resourceName string) (*statz.Counter1[string], error) {
	telemetryDescription := fmt.Sprintf("The status (%s|%s|%s).", successfulReadings, lockNotAcquired, unexpectedError)
	counterName := fmt.Sprintf("lidarReadingsConsumed-%s", resourceName)
	lidarReadingCounter := statz.NewCounter1[string](counterName, statz.MetricConfig{
		Description: "The number of lidar readings",
		Unit:        units.Dimensionless,
		Labels: []statz.Label{
			{
				Name:        "status",
				Description: telemetryDescription,
			},
		},
	})

	return &lidarReadingCounter, nil
}

// addSensorReading adds a lidar reading to the cartofacade.
func addSensorReading(
	ctx context.Context,
	config Config,
) {
	tsr, err := config.Lidar.TimedSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		return
	}

	if tsr.Replay {
		addSensorReadingFromReplaySensor(ctx, tsr.Reading, tsr.ReadingTime, config)
	} else {
		timeToSleep := addSensorReadingFromLiveReadings(ctx, tsr.Reading, tsr.ReadingTime, config)
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
			err := config.CartoFacade.AddSensorReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
			if err == nil {
				if config.TelemetryEnabled {
					config.lidarReadingCounter.Inc(successfulReadings)
				}
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err) // Remove?
				if config.TelemetryEnabled {
					config.lidarReadingCounter.Inc(unexpectedError)
				}
			} else {
				config.Logger.Debugw("Skipping sensor reading due to lock contention in cartofacade", "error", err) // Remove?
				if config.TelemetryEnabled {
					config.lidarReadingCounter.Inc(lockNotAcquired)
				}
			}
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
			config.Logger.Debugw("Skipping sensor reading due to lock contention in cartofacade", "error", err) // Remove?
			if config.TelemetryEnabled {
				config.lidarReadingCounter.Inc(lockNotAcquired)
			}
		} else {
			config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err) // Remove?
			if config.TelemetryEnabled {
				config.lidarReadingCounter.Inc(unexpectedError)
			}
		}
	} else {
		if config.TelemetryEnabled {
			config.lidarReadingCounter.Inc(successfulReadings)
		}
	}

	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(config.DataRateMs-timeElapsedMs)))
}
