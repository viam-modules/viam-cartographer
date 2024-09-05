// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"sort"
	"strings"
	"time"

	"go.viam.com/rdk/components/camera/replaypcd"
	replaymovementsensor "go.viam.com/rdk/components/movementsensor/replay"
	"go.viam.com/rdk/logging"

	"github.com/viam-modules/viam-cartographer/cartofacade"
	s "github.com/viam-modules/viam-cartographer/sensors"
)

type sensorType int64

const (
	lidar sensorType = iota
	movementSensor
)

type offlineSensorReadingTime struct {
	sensorType  sensorType
	readingTime time.Time
}

// Config holds config needed throughout the process of adding a sensor reading to the cartofacade.
type Config struct {
	CartoFacade cartofacade.Interface
	IsOnline    bool

	Lidar          s.TimedLidar
	MovementSensor s.TimedMovementSensor

	Timeout         time.Duration
	InternalTimeout time.Duration
	Logger          logging.Logger
}

// getInitialMovementSensorReading gets the initial movement sensor reading.
// It discards all movement sensor readings that were recorded before the first lidar reading.
func (config *Config) getInitialMovementSensorReading(ctx context.Context,
	lidarReading s.TimedLidarReadingResponse,
) (s.TimedMovementSensorReadingResponse, error) {
	if config.MovementSensor == nil || (!config.MovementSensor.Properties().IMUSupported &&
		!config.MovementSensor.Properties().OdometerSupported) {
		return s.TimedMovementSensorReadingResponse{}, errors.New("movement sensor is not supported")
	}
	for {
		movementSensorReading, err := config.MovementSensor.TimedMovementSensorReading(ctx)
		if err != nil {
			return s.TimedMovementSensorReadingResponse{}, err
		}

		var readingTime time.Time
		if config.MovementSensor.Properties().OdometerSupported {
			// we can assume that the odometer reading time is earlier than the imu reading
			// time, since the odometer reading is taken before the imu reading
			readingTime = movementSensorReading.TimedOdometerResponse.ReadingTime
		} else {
			// we reach this case if the odometer is not supported
			readingTime = movementSensorReading.TimedIMUResponse.ReadingTime
		}

		if !readingTime.Before(lidarReading.ReadingTime) {
			return movementSensorReading, nil
		}
	}
}

// StartOfflineSensorProcess starts the process of adding lidar and movement sensor data
// in a deterministically defined order to cartographer. Returns a bool that indicates
// whether or not the end of either the lidar or movement sensor datasets have been reached.
func (config *Config) StartOfflineSensorProcess(ctx context.Context) bool {
	// get the initial lidar reading
	lidarReading, err := config.Lidar.TimedLidarReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		return strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error())
	}

	var movementSensorReading s.TimedMovementSensorReadingResponse
	if config.MovementSensor != nil && (config.MovementSensor.Properties().IMUSupported ||
		config.MovementSensor.Properties().OdometerSupported) {
		// get the initial IMU reading; discard all IMU readings that were recorded before the first lidar reading
		movementSensorReading, err = config.getInitialMovementSensorReading(ctx, lidarReading)
		if err != nil {
			config.Logger.Warn(err)
			return strings.Contains(err.Error(), replaymovementsensor.ErrEndOfDataset.Error())
		}
	}

	// loop over all the data until one of the datasets has reached its end
	for {
		select {
		case <-ctx.Done():
			return false
		default:
			// create a map of supported sensors and their reading time stamps
			readingTimes := []offlineSensorReadingTime{
				{sensorType: lidar, readingTime: lidarReading.ReadingTime},
			}
			// default to the slightly later imu timestamp: in case that the odometer time stamp was
			// taken before the lidar time stamp, but the imu time stamp was taken after the lidar time
			// stamp, we'll want to prioritize adding the lidar measurement before adding the movement
			// sensor measurement
			if config.MovementSensor != nil && config.MovementSensor.Properties().IMUSupported {
				readingTimes = append(readingTimes,
					offlineSensorReadingTime{
						sensorType:  movementSensor,
						readingTime: movementSensorReading.TimedIMUResponse.ReadingTime,
					})
			} else if config.MovementSensor != nil && config.MovementSensor.Properties().OdometerSupported {
				readingTimes = append(readingTimes,
					offlineSensorReadingTime{
						sensorType:  movementSensor,
						readingTime: movementSensorReading.TimedOdometerResponse.ReadingTime,
					})
			}

			// sort the readings based on their time stamp
			sort.Slice(readingTimes,
				func(i, j int) bool {
					// if the timestamps are the same, we want to prioritize the lidar measurement before
					// the movement sensor measurement
					if readingTimes[i].readingTime.Equal(readingTimes[j].readingTime) {
						return readingTimes[i].sensorType == lidar
					}
					return readingTimes[i].readingTime.Before(readingTimes[j].readingTime)
				})

			// insert the reading with the earliest time stamp
			switch readingTimes[0].sensorType {
			case lidar:
				if err := config.tryAddLidarReadingUntilSuccess(ctx, lidarReading); err != nil {
					return false
				}

				lidarReading, err = config.Lidar.TimedLidarReading(ctx)
				if err != nil {
					config.Logger.Warn(err)
					lidarEndOfDataSetReached := strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error())
					if lidarEndOfDataSetReached {
						config.runFinalOptimization(ctx)
					}
					return lidarEndOfDataSetReached
				}
			case movementSensor:
				if err := config.tryAddMovementSensorReadingUntilSuccess(ctx, movementSensorReading); err != nil {
					return false
				}
				movementSensorReading, err = config.MovementSensor.TimedMovementSensorReading(ctx)
				if err != nil {
					config.Logger.Warn(err)
					msEndOfDataSetReached := strings.Contains(err.Error(), replaymovementsensor.ErrEndOfDataset.Error())
					if msEndOfDataSetReached {
						config.runFinalOptimization(ctx)
					}
					return msEndOfDataSetReached
				}
			}
		}
	}
}

func (config *Config) runFinalOptimization(ctx context.Context) {
	config.Logger.Info("Beginning final optimization")
	if err := config.CartoFacade.RunFinalOptimization(ctx, config.InternalTimeout); err != nil {
		config.Logger.Error("Failed to finish processing all sensor readings: ", err)
	}
}
