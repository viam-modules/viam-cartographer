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
	currentLidarData *s.TimedLidarReadingResponse

	IMU            s.TimedMovementSensor
	currentIMUData *s.TimedIMUReadingResponse

	Timeout                  time.Duration
	InternalTimeout          time.Duration
	Logger                   logging.Logger
	RunFinalOptimizationFunc func(context.Context, time.Duration) error

	Mutex *sync.Mutex
}

// Update currentIMUData under a mutex lock.
func (config *Config) updateMutexProtectedIMUData(data s.TimedIMUReadingResponse) {
	config.Mutex.Lock()
	config.currentIMUData = &data
	config.Mutex.Unlock()
}

// Update currentLidarData under a mutex lock.
func (config *Config) updateMutexProtectedLidarData(data s.TimedLidarReadingResponse) {
	config.Mutex.Lock()
	config.currentLidarData = &data
	config.Mutex.Unlock()
}
