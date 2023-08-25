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
	"go.viam.com/rdk/components/camera/replaypcd"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors"
)

var undefinedIMU = cartofacade.IMUReading{}

// Config holds config needed throughout the process of adding a sensor reading to the cartofacade.
type Config struct {
	LogFile           *os.File
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
		}
	}
}

// addLidarReading adds a lidar reading to the cartofacade.
func (config *Config) addLidarReading(ctx context.Context) bool {
	/*
	 when the lidar data rate msec is non-zero, we assume the user wants to be in "online"
	 mode and ensure the most recent scan gets processed by cartographer. If data rate msec
	 is zero we process every scan in order
	*/
	if config.LidarDataRateMsec != 0 {
		// get next lidar data response
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
	} else {
		// only add the stored lidar data and begin processing the next one, if the stored imu data occurs after it.
		if config.IMUName == "" || config.CurrentData.lidarTime.Sub(config.CurrentData.imuTime).Milliseconds() <= 0 {
			if config.CurrentData.lidarData != nil {
				tryAddLidarReadingUntilSuccess(ctx, config.CurrentData.lidarData, config.CurrentData.lidarTime, *config)
			}
			// get next lidar data response
			tsr, err := config.Lidar.TimedLidarSensorReading(ctx)
			if err != nil {
				config.Logger.Warn(err)
				// only end the sensor process if we are in offline mode
				if config.LidarDataRateMsec == 0 {
					return strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error())
				}
				return false
			}
			// update current lidar data and time
			if config.CurrentData.lidarTime.Sub(tsr.ReadingTime).Milliseconds() < 0 {
				config.CurrentData.lidarTime = tsr.ReadingTime
				config.CurrentData.lidarData = tsr.Reading
			} else {
				config.LogFile.Write([]byte(fmt.Sprintf("%v \t | LIDAR | Dropping data \t \t | %v \n", tsr.ReadingTime, tsr.ReadingTime.Unix())))
			}
			// time.Sleep(time.Millisecond)
		} else {
			time.Sleep(time.Millisecond)
		}
	}
	return false
}

// tryAddLidarReadingUntilSuccess adds a reading to the cartofacade retries on error (offline mode).
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
				config.LogFile.Write([]byte(fmt.Sprintf("%v \t | LIDAR | Success \t \t | %v \n", readingTime, readingTime.Unix())))
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
			}
			config.LogFile.Write([]byte(fmt.Sprintf("%v \t | LIDAR | Failure \t \t | %v \n", readingTime, readingTime.Unix())))
		}
	}
}

// tryAddLidarReading adds a reading to the carto facade does not retry (online).
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
	 when the lidar data rate msec is non-zero, we assume the user wants to be in "online"
	 mode and ensure the most recent scan gets processed by cartographer. If data rate msec
	 is zero we process every scan in order
	*/
	if config.LidarDataRateMsec != 0 {
		// get next imu data response
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
	} else {
		// only add the stored imu data and begin processing the next one, if the stored lidar data occurs after it.
		if config.CurrentData.imuTime.Sub(config.CurrentData.lidarTime).Milliseconds() < 0 {
			if config.CurrentData.imuData != undefinedIMU {
				tryAddIMUReadingUntilSuccess(ctx, config.CurrentData.imuData, config.CurrentData.imuTime, *config)
			}
			// get next imu data response
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

			// parse imu reading
			sr := cartofacade.IMUReading{
				LinearAcceleration: tsr.LinearAcceleration,
				AngularVelocity:    tsr.AngularVelocity,
			}

			// update current imu data and time
			if config.CurrentData.imuTime.Sub(tsr.ReadingTime).Milliseconds() < 0 {
				config.CurrentData.imuTime = tsr.ReadingTime
				config.CurrentData.imuData = sr
			} else {
				config.LogFile.Write([]byte(fmt.Sprintf("%v \t | LIDAR | Dropping data \t \t | %v \n", tsr.ReadingTime, tsr.ReadingTime.Unix())))
			}
			// time.Sleep(time.Millisecond)
		} else {
			time.Sleep(time.Millisecond)
		}
	}
	return false
}

// tryAddIMUReadingUntilSuccess adds a reading to the cartofacade retries on error (offline mode).
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
				config.LogFile.Write([]byte(fmt.Sprintf("%v \t |  IMU  | Success \t \t | %v \n", readingTime, readingTime.Unix())))
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
			}
			config.LogFile.Write([]byte(fmt.Sprintf("%v \t |  IMU  | Failure \t \t | %v \n", readingTime, readingTime.Unix())))
		}
	}
}

// tryAddIMUReading adds a reading to the carto facade does not retry (online).
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
