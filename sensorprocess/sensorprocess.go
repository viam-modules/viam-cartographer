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
	CartoFacade            cartofacade.Interface
	sensorProcessStartTime time.Time
	Online                 bool

	Lidar            s.TimedLidar
	currentLidarData LidarData

	IMU            s.TimedMovementSensor
	currentIMUData IMUData

	Timeout                  time.Duration
	InternalTimeout          time.Duration
	Logger                   logging.Logger
	RunFinalOptimizationFunc func(context.Context, time.Duration) error

	Mutex *sync.Mutex
}

// IMUData stores the next data to be added to cartographer along with its associated timestamp so that,
// in offline mode, data from multiple sensors can be added in order.
type IMUData struct {
	time time.Time
	data cartofacade.IMUReading
}

// LidarData stores the next data to be added to cartographer along with its associated timestamp so that,
// in offline mode, data from multiple sensors can be added in order.
type LidarData struct {
	time time.Time
	data []byte
}

// Update currentIMUData under a mutex lock.
func (config *Config) updateMutexProtectedIMUData(time time.Time, data cartofacade.IMUReading) {
	config.Mutex.Lock()
	config.currentIMUData.time = time
	config.currentIMUData.data = data
	config.Mutex.Unlock()
}

// Update currentLidarData under a mutex lock.
func (config *Config) updateMutexProtectedLidarData(time time.Time, data []byte) {
	config.Mutex.Lock()
	config.currentLidarData.time = time
	config.currentLidarData.data = data
	config.Mutex.Unlock()
}
