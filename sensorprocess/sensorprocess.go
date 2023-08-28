// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's cartofacade
package sensorprocess

import (
	"context"
	"errors"
	"math"
	"strings"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/components/camera/replaypcd"
	replaymovementsensor "go.viam.com/rdk/components/movementsensor/replay"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors"
)

var undefinedIMU = cartofacade.IMUReading{}

// Config holds config needed throughout the process of adding a sensor reading to the cartofacade.
type Config struct {
	CartoFacade              cartofacade.Interface
	Lidar                    sensors.TimedLidarSensor
	LidarName                string
	LidarDataRateMsec        int
	IMU                      sensors.TimedIMUSensor
	IMUName                  string
	IMUDataRateMsec          int
	Timeout                  time.Duration
	Logger                   golog.Logger
	nextData                 nextData
	started                  bool
	stopped                  bool
	RunFinalOptimizationFunc func(context.Context, time.Duration) error
}

// nextData stores the next data to be added to cartographer along with its associated timestamp so that,
// in offline mode, data from multiple sensors can be added in order.
type nextData struct {
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
				config.stopped = true
				config.Logger.Info("Beginning final optimization")
				err := config.RunFinalOptimizationFunc(ctx, config.Timeout)
				if err != nil {
					config.Logger.Error("Failed to finish processing all sensor readings")
				}
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
		tsr, status, err := getTimedLidarSensorReading(ctx, config)
		if err != nil {
			return status
		}

		// sleep remainder of time interval
		timeToSleep := tryAddLidarReading(ctx, tsr.Reading, tsr.ReadingTime, *config)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
		config.Logger.Debugf("sleep for %s milliseconds", time.Duration(timeToSleep))
	} else {
		// add the stored lidar data and begin processing the next one, if no imu exists or the currently stored imu data occurs after it.
		if config.IMUName == "" || config.nextData.lidarTime.Sub(config.nextData.imuTime).Milliseconds() <= 0 {
			if config.nextData.lidarData != nil {
				tryAddLidarReadingUntilSuccess(ctx, config.nextData.lidarData, config.nextData.lidarTime, *config)
				config.started = true
			}
			// get next lidar data response
			tsr, status, err := getTimedLidarSensorReading(ctx, config)
			if err != nil {
				return status
			}

			config.nextData.lidarTime = tsr.ReadingTime
			config.nextData.lidarData = tsr.Reading
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
				config.Logger.Debugf("%v \t | LIDAR | Success \t \t | %v \n", readingTime, readingTime.Unix())
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
			}
			config.Logger.Debugf("%v \t | LIDAR | Failure \t \t | %v \n", readingTime, readingTime.Unix())
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
			if config.stopped {
				return true
			}
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
		tsr, status, err := getTimedIMUSensorReading(ctx, config)
		if err != nil {
			return status
		}

		// parse imu reading
		sr := cartofacade.IMUReading{
			LinearAcceleration: tsr.LinearAcceleration,
			AngularVelocity:    tsr.AngularVelocity,
		}

		// sleep remainder of time interval
		timeToSleep := tryAddIMUReading(ctx, sr, tsr.ReadingTime, *config)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	} else {
		// add the stored imu data and begin processing the next one, if the currently stored lidar data occurs after it.
		if config.nextData.imuTime.Sub(config.nextData.lidarTime).Milliseconds() < 0 {
			if config.started && config.nextData.imuData != undefinedIMU {
				tryAddIMUReadingUntilSuccess(ctx, config.nextData.imuData, config.nextData.imuTime, *config)
			}
			// get next imu data response
			tsr, status, err := getTimedIMUSensorReading(ctx, config)
			if err != nil {
				return status
			}

			// parse imu reading
			sr := cartofacade.IMUReading{
				LinearAcceleration: tsr.LinearAcceleration,
				AngularVelocity:    tsr.AngularVelocity,
			}

			// TODO: Remove dropping out of order imu readings after DATA-1812 has been complete
			// JIRA Ticket: https://viam.atlassian.net/browse/DATA-1812
			// update current imu data and time
			if config.nextData.imuTime.Sub(tsr.ReadingTime).Milliseconds() < 0 {
				config.nextData.imuTime = tsr.ReadingTime
				config.nextData.imuData = sr
			} else {
				config.Logger.Debugf("%v \t | IMU | Dropping data \t \t | %v \n", tsr.ReadingTime, tsr.ReadingTime.Unix())
			}
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
				config.Logger.Debugf("%v \t |  IMU  | Success \t \t | %v \n", readingTime, readingTime.Unix())
				return
			}
			if !errors.Is(err, cartofacade.ErrUnableToAcquireLock) {
				config.Logger.Warnw("Retrying sensor reading due to error from cartofacade", "error", err)
			}
			config.Logger.Debugf("%v \t |  IMU  | Failure \t \t | %v \n", readingTime, readingTime.Unix())
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

// getTimedIMUSensorReading returns the next imu reading if available along with a status denoting if the end of dataset as been reached.
func getTimedIMUSensorReading(ctx context.Context, config *Config) (sensors.TimedIMUSensorReadingResponse, bool, error) {
	tsr, err := config.IMU.TimedIMUSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		// only end the sensor process if we are in offline mode
		if config.LidarDataRateMsec == 0 {
			return tsr, strings.Contains(err.Error(), replaymovementsensor.ErrEndOfDataset.Error()), err
		}
		return tsr, false, err
	}
	return tsr, false, err
}

// getTimedLidarSensorReading returns the next lidar reading if available along with a status denoting if the end of dataset as been
// reached.
func getTimedLidarSensorReading(ctx context.Context, config *Config) (sensors.TimedLidarSensorReadingResponse, bool, error) {
	tsr, err := config.Lidar.TimedLidarSensorReading(ctx)
	if err != nil {
		config.Logger.Warn(err)
		// only end the sensor process if we are in offline mode
		if config.LidarDataRateMsec == 0 {
			return tsr, strings.Contains(err.Error(), replaypcd.ErrEndOfDataset.Error()), err
		}
		return tsr, false, err
	}
	return tsr, false, err
}
