// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"sync"
	"time"

	"go.viam.com/rdk/logging"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
)

// defaultTime is used to check if timestamps have not been set yet.
var defaultTime = time.Time{}

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

	Mutex *sync.Mutex
}

func (config *Config) StartOfflineSensorProcess(ctx context.Context) bool {
	// get the initial lidar reading
	lidarReading, lidarEndOfDataSetReached, err := getTimedLidarReading(ctx, config)
	if err != nil || lidarEndOfDataSetReached {
		return lidarEndOfDataSetReached
	}

	// get the initial IMU reading; discard all IMU readings that were recorded before the first lidar reading
	var imuReading s.TimedIMUReadingResponse
	for {
		msReading, msEndOfDataSetReached, err := getTimedMovementSensorReading(ctx, config)
		if err != nil || msEndOfDataSetReached {
			return msEndOfDataSetReached
		}
		imuReading = *msReading.TimedIMUResponse
		if imuReading.ReadingTime.After(lidarReading.ReadingTime) {
			break
		}
	}

	// loop over all the data until one of the datasets has reached its end
	for {
		select {
		case <-ctx.Done():
			return false
		default:
			// insert the reading with the earliest time stamp
			if lidarReading.ReadingTime.Before(imuReading.ReadingTime) || lidarReading.ReadingTime.Equal(imuReading.ReadingTime) {
				config.tryAddLidarReadingUntilSuccess(ctx, lidarReading)
				lidarReading, lidarEndOfDataSetReached, err = getTimedLidarReading(ctx, config)
				if err != nil || lidarEndOfDataSetReached {
					return lidarEndOfDataSetReached
				}
			} else {
				config.tryAddIMUReadingUntilSuccess(ctx, imuReading)
				msReading, msEndOfDataSetReached, err := getTimedMovementSensorReading(ctx, config)
				if err != nil || msEndOfDataSetReached {
					return msEndOfDataSetReached
				}
				imuReading = *msReading.TimedIMUResponse
			}
		}
	}
}
