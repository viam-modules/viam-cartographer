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

var (
	undefinedIMU = cartofacade.IMUReading{}
	defaultTime  = time.Time{}
)

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
	nextLidarData            nextLidarData
	nextIMUData              nextIMUData
	firstLidarReadingTime    time.Time
	lastLidarReadingTime     time.Time
	RunFinalOptimizationFunc func(context.Context, time.Duration) error
}

// nextData stores the next data to be added to cartographer along with its associated timestamp so that,
// in offline mode, data from multiple sensors can be added in order.
type nextLidarData struct {
	time time.Time
	data []byte
}

type nextIMUData struct {
	time time.Time
	data cartofacade.IMUReading
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
				config.lastLidarReadingTime = config.nextLidarData.time
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
		/*
			In order for cartographer to build a correct map, the lidar and imu readings need to be processed in
			order in offline mode. We only add the stored lidar data if we do not have any IMU data to add, or if
			the next IMU data has a timestamp after the current lidar reading's timestamp.
		*/
		if config.IMUName == "" || config.nextLidarData.time.Sub(config.nextIMUData.time).Milliseconds() <= 0 {
			if config.nextLidarData.data != nil {
				tryAddLidarReadingUntilSuccess(ctx, config.nextLidarData.data, config.nextLidarData.time, *config)
				if config.firstLidarReadingTime == defaultTime {
					config.firstLidarReadingTime = config.nextLidarData.time
				}
			}
			// get next lidar data response
			tsr, status, err := getTimedLidarSensorReading(ctx, config)
			if err != nil {
				return status
			}

			config.nextLidarData.time = tsr.ReadingTime
			config.nextLidarData.data = tsr.Reading
		} else {
			time.Sleep(time.Millisecond)
		}
	}
	return false
}

// tryAddLidarReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode).
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

// tryAddLidarReading adds a reading to the carto facade and does not retry (online).
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
			if config.lastLidarReadingTime != defaultTime && config.nextIMUData.time.Sub(config.lastLidarReadingTime) > 0 {
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
		/*
			In order for cartographer to build a correct map, the lidar and imu readings need to be processed in
			order in offline mode. We only add the stored IMU data if the next lidar data has a timestamp after
			the current IMU reading's timestamp.
		*/
		if config.nextIMUData.time.Sub(config.nextLidarData.time).Milliseconds() < 0 {
			if config.nextIMUData.data != undefinedIMU && config.firstLidarReadingTime != defaultTime &&
				config.nextIMUData.time.Sub(config.firstLidarReadingTime) > 0 {
				tryAddIMUReadingUntilSuccess(ctx, config.nextIMUData.data, config.nextIMUData.time, *config)
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
			if config.nextIMUData.time.Sub(tsr.ReadingTime).Milliseconds() < 0 {
				config.nextIMUData.time = tsr.ReadingTime
				config.nextIMUData.data = sr
			} else {
				config.Logger.Debugf("%v \t | IMU | Dropping data \t \t | %v \n", tsr.ReadingTime, tsr.ReadingTime.Unix())
			}
		} else {
			time.Sleep(time.Millisecond)
		}
	}
	return false
}

// tryAddIMUReadingUntilSuccess adds a reading to the cartofacade and retries on error (offline mode).
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

// tryAddIMUReading adds a reading to the carto facade and does not retry (online).
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

// getTimedIMUSensorReading returns the next imu reading if available along with a status denoting if the end of dataset has been reached.
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

// getTimedLidarSensorReading returns the next lidar reading if available along with a status denoting if the end of dataset has been
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
