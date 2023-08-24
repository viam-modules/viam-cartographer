// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"fmt"
	"math"
	"os"
	"strings"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera/replaypcd"
	"go.viam.com/rdk/spatialmath"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors"
)

// Config holds config needed throughout the process of adding a sensor reading to the cartofacade.
type Config struct {
	File              *os.File
	CartoFacade       cartofacade.Interface
	Lidar             sensors.TimedLidarSensor
	LidarName         string
	LidarDataRateMsec int
	IMU               sensors.TimedIMUSensor
	IMUName           string
	IMUDataRateMsec   int
	Timeout           time.Duration
	Logger            golog.Logger
	CurrentData       currentData
	offsetAV          spatialmath.AngularVelocity
	offsetLA          r3.Vector
}

type currentData struct {
	lidarTime time.Time
	lidarData []byte
	imuTime   time.Time
	imuData   cartofacade.IMUReading
}

// StartLidar polls the lidar to get the next sensor reading and adds it to the cartofacade.
// stops when the context is Done.
func (config *Config) StartLidar(
	ctx context.Context,
) bool {
	for {
		select {
		case <-ctx.Done():
			return false
		default:
			if jobDone := config.addLidarReading(ctx); jobDone {
				return true
			}
			if config.IMUName != "" {
				if jobDone := config.addIMUReading(ctx); jobDone {
					return true
				}
			}
		}
	}
}

// addLidarReading adds a lidar reading to the cartofacade.
func (config *Config) addLidarReading(
	ctx context.Context,
) bool {
	/*
	 when the lidar data rate msec is 0, we assume the user wants to be in "offline"
	 mode and ensure every scan gets processed by cartographer
	*/
	fmt.Printf("1. LIDAR: %v IMU: %v | %v\n", config.CurrentData.lidarTime.Unix(), config.CurrentData.imuTime.Unix(), config.CurrentData.lidarTime.Sub(config.CurrentData.imuTime).Milliseconds())
	if config.LidarDataRateMsec == 0 {
		if config.IMUName == "" || config.CurrentData.lidarTime.Sub(config.CurrentData.imuTime).Milliseconds() <= 0 {
			if len(config.CurrentData.lidarData) != 0 {
				fmt.Printf("Adding current data lidar: %v\n", config.CurrentData.lidarTime.Unix())
				tryAddLidarReadingUntilSuccess(ctx, config.CurrentData.lidarData, config.CurrentData.lidarTime, *config)
			}
			//tryAddLidarReadingUntilSuccess(ctx, tsr.Reading, tsr.ReadingTime, *config)
			tsr, err := config.Lidar.TimedLidarSensorReading(ctx)
			if err != nil {
				config.Logger.Warn(err)
				// only end the sensor process if we are in offline mode
				if config.LidarDataRateMsec == 0 {
					return strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error())
				}
				return false
			}
			//tryAddLidarReadingUntilSuccess(ctx, tsr.Reading, tsr.ReadingTime, *config)
			if config.CurrentData.lidarTime.Sub(tsr.ReadingTime).Milliseconds() < 0 {
				config.CurrentData.lidarTime = tsr.ReadingTime
				config.CurrentData.lidarData = tsr.Reading
				fmt.Printf("Updating current data lidar: %v\n", tsr.ReadingTime.Unix())
			} else {
				fmt.Println("DROPPING LIDAR DATA")
				config.File.Write([]byte(fmt.Sprintf("Dropping LIDAR Data: %v\n", tsr.ReadingTime.Unix())))
			}
			time.Sleep(time.Duration(5) * time.Millisecond)
		}
	} else {
		tsr, err := config.Lidar.TimedLidarSensorReading(ctx)
		if err != nil {
			config.Logger.Warn(err)
			// only end the sensor process if we are in offline mode
			if config.LidarDataRateMsec == 0 {
				return strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error())
			}
			return false
		}

		timeToSleep := tryAddLidarReading(ctx, tsr.Reading, tsr.ReadingTime, *config)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
		config.Logger.Debugf("sleep for %s milliseconds", time.Duration(timeToSleep))
	}
	return false
}

// tryAddLidarReadingUntilSuccess adds a reading to the cartofacade
// retries on error (offline mode).
func tryAddLidarReadingUntilSuccess(ctx context.Context, reading []byte, readingTime time.Time, config Config) {
	/*
		while add lidar reading fails, keep trying to add the same reading - in offline mode
		we want to process each reading so if we cannot acquire the lock we should try again
	*/
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddLidarReading(ctx, config.Timeout, config.LidarName, reading, readingTime)
			if err == nil {
				config.File.Write([]byte(fmt.Sprintf("%v | LIDAR | SUCC \n", readingTime)))
				return
			}
			config.File.Write([]byte(fmt.Sprintf("%v | LIDAR | FAIL \n", readingTime)))
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
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
			config.Logger.Debugw("Skipping lidar reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping lidar reading due to error from cartofacade", "error", err)
		}
	}
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(config.LidarDataRateMsec-timeElapsedMs)))
}

// StartIMU polls the IMU to get the next sensor reading and adds it to the cartofacade.
// stops when the context is Done.
func (config *Config) StartIMU(
	ctx context.Context,
) bool {
	for {
		select {
		case <-ctx.Done():
			return false
		default:
			if jobDone := config.addIMUReading(ctx); jobDone {
				return true
			}
		}
	}
}

// addIMUReading adds an IMU reading to the cartofacade.
func (config *Config) addIMUReading(
	ctx context.Context,
) bool {
	/*
	 when the lidar data rate msec is 0, we assume the user wants to be in "offline"
	 mode and ensure every scan gets processed by cartographer
	*/
	fmt.Printf("2. LIDAR: %v IMU: %v | %v\n", config.CurrentData.lidarTime.Unix(), config.CurrentData.imuTime.Unix(), config.CurrentData.lidarTime.Sub(config.CurrentData.imuTime).Milliseconds())
	if config.LidarDataRateMsec == 0 {
		if config.CurrentData.imuTime.Sub(config.CurrentData.lidarTime).Milliseconds() < 0 {
			var blankIMU cartofacade.IMUReading
			if config.CurrentData.imuData != blankIMU {
				fmt.Printf("Adding current data imu: %v\n", config.CurrentData.imuTime.Unix())
				tryAddIMUReadingUntilSuccess(ctx, config.CurrentData.imuData, config.CurrentData.imuTime, *config)
			}
			tsr, err := config.IMU.TimedIMUSensorReading(ctx)
			// Fully implement once we support replay movementsensors, see https://viam.atlassian.net/browse/RSDK-4111
			if err != nil {
				config.Logger.Warn(err)
				// only end the sensor process if we are in offline mode
				if config.LidarDataRateMsec == 0 {
					if config.IMUDataRateMsec != 0 {
						config.Logger.Warn("In offline mode, but IMU data frequency is nonzero")
					}
					return true
					// return strings.Contains(err.Error(), replaymovementsensor.ErrEndOfDataset.Error())
				}
				return false
			}

			sr := cartofacade.IMUReading{
				LinearAcceleration: tsr.LinearAcceleration,
				AngularVelocity:    tsr.AngularVelocity,
			}

			if config.IMUDataRateMsec != 0 {
				config.Logger.Warn("In offline mode, but IMU data frequency is nonzero")
			}
			//tryAddIMUReadingUntilSuccess(ctx, sr, tsr.ReadingTime, *config)
			if config.CurrentData.imuTime.Sub(tsr.ReadingTime).Milliseconds() < 0 {
				config.CurrentData.imuTime = tsr.ReadingTime
				config.CurrentData.imuData = sr
				fmt.Printf("Updating current data imu: %v\n", tsr.ReadingTime.Unix())
			} else {
				fmt.Println("DROPPING IMU DATA")
				config.File.Write([]byte(fmt.Sprintf("Dropping IMU Data: %v\n", tsr.ReadingTime.Unix())))
			}
			time.Sleep(time.Duration(5) * time.Millisecond)
		}
	} else {
		tsr, err := config.IMU.TimedIMUSensorReading(ctx)
		// Fully implement once we support replay movementsensors, see https://viam.atlassian.net/browse/RSDK-4111
		if err != nil {
			config.Logger.Warn(err)
			// only end the sensor process if we are in offline mode
			if config.LidarDataRateMsec == 0 {
				if config.IMUDataRateMsec != 0 {
					config.Logger.Warn("In offline mode, but IMU data frequency is nonzero")
				}
				return true
				// return strings.Contains(err.Error(), replaymovementsensor.ErrEndOfDataset.Error())
			}
			return false
		}

		sr := cartofacade.IMUReading{
			LinearAcceleration: tsr.LinearAcceleration,
			AngularVelocity:    tsr.AngularVelocity,
		}

		timeToSleep := tryAddIMUReading(ctx, sr, tsr.ReadingTime, *config)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	}
	return false
}

// tryAddIMUReadingUntilSuccess adds a reading to the cartofacade
// retries on error (offline mode).
func tryAddIMUReadingUntilSuccess(ctx context.Context, reading cartofacade.IMUReading, readingTime time.Time, config Config) {
	/*
		while add sensor reading fails, keep trying to add the same reading - in offline mode
		we want to process each reading so if we cannot acquire the lock we should try again
	*/
	for {
		select {
		case <-ctx.Done():
			return
		default:
			err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.IMUName, reading, readingTime)
			if err == nil {
				config.File.Write([]byte(fmt.Sprintf("%v | IMU | SUCC | %v \n", readingTime, reading)))
				return
			}
			config.File.Write([]byte(fmt.Sprintf("%v | IMU | FAIL | \n", readingTime)))
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
			}
		}
	}
}

// tryAddIMUReading adds a reading to the carto facade
// does not retry (online).
func tryAddIMUReading(ctx context.Context, reading cartofacade.IMUReading, readingTime time.Time, config Config) int {
	startTime := time.Now()
	err := config.CartoFacade.AddIMUReading(ctx, config.Timeout, config.IMUName, reading, readingTime)
	if err != nil {
		if errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
			config.Logger.Debugw("Skipping sensor reading due to lock contention in cartofacade", "error", err)
		} else {
			config.Logger.Warnw("Skipping sensor reading due to error from cartofacade", "error", err)
		}
	}
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(config.IMUDataRateMsec-timeElapsedMs)))
}
