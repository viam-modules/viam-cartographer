// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"math"
	"time"

	replaymovementsensor "go.viam.com/rdk/components/movementsensor/replay"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
)

// StartMovementSensor polls the movement sensor to get the next sensor reading
// and adds it to the cartofacade. Stops when the context is Done.
func (config *Config) StartMovementSensor(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
			return
		default:
			if err := config.addMovementSensorReadingInOnline(ctx); err != nil {
				config.Logger.Warn(err)
			}
		}
	}
}

// addMovementSensorReadingInOnline ensures the most recent movement sensor scan,
// after corresponding lidar scans, gets processed by cartographer.
func (config *Config) addMovementSensorReadingInOnline(ctx context.Context) error {
	// get next movement sensor data response; ignoring status since it is always false
	movementSensorReading, err := config.MovementSensor.TimedMovementSensorReading(ctx)
	if err != nil {
		if errors.Is(err, replaymovementsensor.ErrEndOfDataset) {
			time.Sleep(1 * time.Second)
		}
		return err
	}

	// add movement sensor data to cartographer and sleep remainder of time interval
	timeToSleep := config.tryAddMovementSensorReadingOnce(ctx, movementSensorReading)

	if !movementSensorReading.TestIsReplaySensor {
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
		config.Logger.Debugf("movement sensor sleep for %vms", timeToSleep)
	}

	return nil
}

// tryAddMovementSensorReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode).
// While add sensor reading fails, keep trying to add the same reading - in offline mode we want to
// process each reading so if we cannot acquire the lock we should try again.
func (config *Config) tryAddMovementSensorReadingUntilSuccess(ctx context.Context, reading s.TimedMovementSensorReadingResponse) error {
	var imuDone, odometerDone bool
	if !config.MovementSensor.Properties().IMUSupported {
		imuDone = true
	}
	if !config.MovementSensor.Properties().OdometerSupported {
		odometerDone = true
	}
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		default:
			if !imuDone {
				if err := config.tryAddIMUReading(ctx, *reading.TimedIMUResponse); err != nil {
					if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
						config.Logger.Warnw("Retrying IMU sensor reading due to error from cartofacade", "error", err)
					}
				} else {
					imuDone = true
				}
			}
			if !odometerDone {
				if err := config.tryAddOdometerReading(ctx, *reading.TimedOdometerResponse); err != nil {
					if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
						config.Logger.Warnw("Retrying odometer sensor reading due to error from cartofacade", "error", err)
					}
				} else {
					odometerDone = true
				}
			}
			if imuDone && odometerDone {
				return nil
			}
		}
	}
}

// tryAddMovementSensorReadingOnce adds a reading to the carto facade and does not retry. Returns remainder of time interval.
func (config *Config) tryAddMovementSensorReadingOnce(ctx context.Context, reading s.TimedMovementSensorReadingResponse) int {
	startTime := time.Now().UTC()

	if config.MovementSensor.Properties().IMUSupported {
		if err := config.tryAddIMUReading(ctx, *reading.TimedIMUResponse); err != nil {
			if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Debugw("Skipping IMU sensor reading due to lock contention in cartofacade", "error", err)
			} else {
				config.Logger.Warnw("Skipping IMU sensor reading due to error from cartofacade", "error", err)
			}
		}
	}

	if config.MovementSensor.Properties().OdometerSupported {
		if err := config.tryAddOdometerReading(ctx, *reading.TimedOdometerResponse); err != nil {
			if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Debugw("Skipping odometer sensor reading due to lock contention in cartofacade", "error", err)
			} else {
				config.Logger.Warnw("Skipping odometer sensor reading due to error from cartofacade", "error", err)
			}
		}
	}

	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(1000/config.MovementSensor.DataFrequencyHz()-timeElapsedMs)))
}

// tryAddIMUReading tries to add an IMU reading to the carto facade.
func (config *Config) tryAddIMUReading(ctx context.Context, reading s.TimedIMUReadingResponse) error {
	err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.MovementSensor.Name(), reading)
	if err != nil {
		config.Logger.Debugf("%v \t |  IMU  | Failure \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	} else {
		config.Logger.Debugf("%v \t |  IMU  | Success \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	}
	return err
}

// tryAddOdometerReading tries to add an odometer reading to the carto facade.
func (config *Config) tryAddOdometerReading(ctx context.Context, reading s.TimedOdometerReadingResponse) error {
	err := config.CartoFacade.AddOdometerReading(ctx, config.Timeout, config.MovementSensor.Name(), reading)
	if err != nil {
		config.Logger.Debugf("%v \t |  Odometer  | Failure \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	} else {
		config.Logger.Debugf("%v \t |  Odometer  | Success \t \t | %v \n", reading.ReadingTime, reading.ReadingTime.Unix())
	}
	return err
}
