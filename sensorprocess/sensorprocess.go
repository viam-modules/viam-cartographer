// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"time"

	"go.viam.com/rdk/logging"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
)

// Config holds config needed throughout the process of adding a sensor reading to the cartofacade.
type Config struct {
	CartoFacade cartofacade.Interface
	IsOnline    bool

	Lidar s.TimedLidar
	IMU   s.TimedMovementSensor

	Timeout                  time.Duration
	InternalTimeout          time.Duration
	Logger                   logging.Logger
	RunFinalOptimizationFunc func(context.Context, time.Duration) error
}

// StartOfflineSensorProcess TODO[kat].
func (config *Config) StartOfflineSensorProcess(ctx context.Context) bool {
	// get the initial lidar reading
	lidarReading, lidarEndOfDataSetReached, err := getTimedLidarReading(ctx, config)
	if err != nil || lidarEndOfDataSetReached {
		return lidarEndOfDataSetReached
	}

	var imuReading s.TimedIMUReadingResponse
	if config.IMU != nil {
		// get the initial IMU reading; discard all IMU readings that were recorded before the first lidar reading
		for {
			msReading, msEndOfDataSetReached, err := getTimedMovementSensorReading(ctx, config)
			if err != nil || msEndOfDataSetReached {
				return msEndOfDataSetReached
			}
			imuReading = *msReading.TimedIMUResponse
			if imuReading.ReadingTime.Equal(lidarReading.ReadingTime) ||
				imuReading.ReadingTime.After(lidarReading.ReadingTime) {
				break
			}
		}
	}

	// loop over all the data until one of the datasets has reached its end
	for {
		select {
		case <-ctx.Done():
			return false
		default:
			// insert the reading with the earliest time stamp
			if config.IMU == nil ||
				lidarReading.ReadingTime.Before(imuReading.ReadingTime) ||
				lidarReading.ReadingTime.Equal(imuReading.ReadingTime) {
				config.tryAddLidarReadingUntilSuccess(ctx, lidarReading)
				lidarReading, lidarEndOfDataSetReached, err = getTimedLidarReading(ctx, config)
				if err != nil || lidarEndOfDataSetReached {
					if lidarEndOfDataSetReached {
						config.runFinalOptimization(ctx)
					}
					return lidarEndOfDataSetReached
				}
			} else {
				config.tryAddIMUReadingUntilSuccess(ctx, imuReading)
				msReading, msEndOfDataSetReached, err := getTimedMovementSensorReading(ctx, config)
				if err != nil || msEndOfDataSetReached {
					if msEndOfDataSetReached {
						config.runFinalOptimization(ctx)
					}
					return msEndOfDataSetReached
				}
				imuReading = *msReading.TimedIMUResponse
			}
		}
	}
}

func (config *Config) runFinalOptimization(ctx context.Context) {
	config.Logger.Info("Beginning final optimization")
	if err := config.RunFinalOptimizationFunc(ctx, config.InternalTimeout); err != nil {
		config.Logger.Error("Failed to finish processing all sensor readings: ", err)
	}
}
