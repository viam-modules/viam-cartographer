package sensorprocess

import (
	"context"
	"errors"
	"fmt"
	"strings"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"go.viam.com/rdk/components/camera/replaypcd"
	"go.viam.com/rdk/components/movementsensor/replay"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/inject"
)

func TestGetInitialMovementSensorReading(t *testing.T) {
	logger := logging.NewTestLogger(t)

	lidarReading := s.TimedLidarReadingResponse{
		Reading:     []byte("12345"),
		ReadingTime: time.Now().UTC(),
	}

	odometerReading := s.TimedOdometerReadingResponse{
		Position:    geo.NewPoint(5, 4),
		Orientation: &spatialmath.Quaternion{Real: 0.1, Imag: -0.2, Jmag: 2.5, Kmag: -9.1},
		ReadingTime: time.Now().UTC(),
	}

	imuReading := s.TimedIMUReadingResponse{
		LinearAcceleration: r3.Vector{X: 1, Y: 2, Z: 3},
		AngularVelocity:    spatialmath.AngularVelocity{X: 4, Y: 5, Z: 6},
		ReadingTime:        time.Now().UTC(),
	}

	cf := cartofacade.Mock{}

	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return 0 }

	injectMovementSensor := inject.TimedMovementSensor{}
	injectMovementSensor.NameFunc = func() string { return "good_movement_sensor" }
	injectMovementSensor.DataFrequencyHzFunc = func() int { return 0 }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}

	t.Run("return error if movement sensor is not supported", func(t *testing.T) {
		config.MovementSensor = nil
		movementSensorReading, err := config.getInitialMovementSensorReading(context.Background(), lidarReading)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeError, errors.New("movement sensor is not supported"))
		test.That(t, movementSensorReading, test.ShouldResemble, s.TimedMovementSensorReadingResponse{})
	})

	t.Run("return error if neither IMU nor odometer are supported", func(t *testing.T) {
		injectMovementSensor.PropertiesFunc = func() s.MovementSensorProperties {
			return s.MovementSensorProperties{}
		}
		config.MovementSensor = &injectMovementSensor
		movementSensorReading, err := config.getInitialMovementSensorReading(context.Background(), lidarReading)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeError, errors.New("movement sensor is not supported"))
		test.That(t, movementSensorReading, test.ShouldResemble, s.TimedMovementSensorReadingResponse{})
	})

	t.Run("return error if TimedMovementSensorReading errors out", func(t *testing.T) {
		expectedErr := errors.New("error in TimedMovementSensorReading")
		injectMovementSensor.PropertiesFunc = func() s.MovementSensorProperties {
			return s.MovementSensorProperties{
				IMUSupported: true,
			}
		}
		injectMovementSensor.TimedMovementSensorReadingFunc = func(ctx context.Context) (s.TimedMovementSensorReadingResponse, error) {
			return s.TimedMovementSensorReadingResponse{}, expectedErr
		}
		config.MovementSensor = &injectMovementSensor
		movementSensorReading, err := config.getInitialMovementSensorReading(context.Background(), lidarReading)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err, test.ShouldBeError, expectedErr)
		test.That(t, movementSensorReading, test.ShouldResemble, s.TimedMovementSensorReadingResponse{})
	})

	t.Run("successful data insertion", func(t *testing.T) {
		cases := []struct {
			description                  string
			imuEnabled                   bool
			odometerEnabled              bool
			lidarReadingTimeAddedMs      int
			msReadingTimeAddedMs         []int
			expectedMsReadingTimeAddedMs int
		}{
			{
				description:                  "skip movement sensor data until first lidar data is inserted",
				imuEnabled:                   true,
				odometerEnabled:              true,
				lidarReadingTimeAddedMs:      5,
				msReadingTimeAddedMs:         []int{1, 2, 3, 4, 6, 7, 8},
				expectedMsReadingTimeAddedMs: 6,
			},
			{
				description:                  "skip imu data until first lidar data is inserted",
				imuEnabled:                   true,
				odometerEnabled:              false,
				lidarReadingTimeAddedMs:      7,
				msReadingTimeAddedMs:         []int{1, 2, 3, 4, 5, 6, 7, 8},
				expectedMsReadingTimeAddedMs: 7,
			},
			{
				description:                  "skip odometer data until first lidar data is inserted",
				imuEnabled:                   false,
				odometerEnabled:              true,
				lidarReadingTimeAddedMs:      3,
				msReadingTimeAddedMs:         []int{1, 2, 5, 6, 7, 8},
				expectedMsReadingTimeAddedMs: 5,
			},
		}

		for _, tt := range cases {
			t.Run(tt.description, func(t *testing.T) {
				now := time.Now().UTC()

				if tt.imuEnabled || tt.odometerEnabled {
					config.MovementSensor = &injectMovementSensor
				} else {
					config.MovementSensor = nil
				}

				lidarReading.ReadingTime = now.Add(time.Duration(tt.lidarReadingTimeAddedMs) * time.Millisecond)

				numMovementSensorData := 0
				injectMovementSensor.TimedMovementSensorReadingFunc = func(ctx context.Context) (s.TimedMovementSensorReadingResponse, error) {
					var movementSensorReading s.TimedMovementSensorReadingResponse
					if numMovementSensorData < len(tt.msReadingTimeAddedMs) {
						if tt.odometerEnabled {
							movementSensorReading.TimedOdometerResponse = &odometerReading
							movementSensorReading.TimedOdometerResponse.ReadingTime = now.Add(time.Duration(tt.msReadingTimeAddedMs[numMovementSensorData]) * time.Millisecond)
						}
						if tt.imuEnabled {
							movementSensorReading.TimedIMUResponse = &imuReading
							movementSensorReading.TimedIMUResponse.ReadingTime = now.Add(time.Duration(float64(tt.msReadingTimeAddedMs[numMovementSensorData])+0.1) * time.Millisecond)
						}
						numMovementSensorData++
						return movementSensorReading, nil
					}
					return movementSensorReading, replay.ErrEndOfDataset
				}
				injectMovementSensor.PropertiesFunc = func() s.MovementSensorProperties {
					return s.MovementSensorProperties{
						IMUSupported:      tt.imuEnabled,
						OdometerSupported: tt.odometerEnabled,
					}
				}

				movementSensorReading, err := config.getInitialMovementSensorReading(context.Background(), lidarReading)
				test.That(t, err, test.ShouldBeNil)
				test.That(t, movementSensorReading, test.ShouldNotResemble, s.TimedMovementSensorReadingResponse{})
				test.That(t, tt.msReadingTimeAddedMs[numMovementSensorData-1], test.ShouldResemble, tt.expectedMsReadingTimeAddedMs)
			})
		}
	})
}

func TestStartOfflineSensorProcess(t *testing.T) {
	logger := logging.NewTestLogger(t)

	lidarReading := s.TimedLidarReadingResponse{
		Reading:     []byte("12345"),
		ReadingTime: time.Now().UTC(),
	}

	odometerReading := s.TimedOdometerReadingResponse{
		Position:    geo.NewPoint(5, 4),
		Orientation: &spatialmath.Quaternion{Real: 0.1, Imag: -0.2, Jmag: 2.5, Kmag: -9.1},
		ReadingTime: time.Now().UTC(),
	}

	imuReading := s.TimedIMUReadingResponse{
		LinearAcceleration: r3.Vector{X: 1, Y: 2, Z: 3},
		AngularVelocity:    spatialmath.AngularVelocity{X: 4, Y: 5, Z: 6},
		ReadingTime:        time.Now().UTC(),
	}

	movementSensorReading := s.TimedMovementSensorReadingResponse{
		TimedIMUResponse:      &imuReading,
		TimedOdometerResponse: &odometerReading,
	}

	cf := cartofacade.Mock{}

	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return 0 }

	injectMovementSensor := inject.TimedMovementSensor{}
	injectMovementSensor.NameFunc = func() string { return "good_movement_sensor" }
	injectMovementSensor.DataFrequencyHzFunc = func() int { return 0 }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}

	t.Run("no data is added if lidar reached end of data set at the start", func(t *testing.T) {
		injectLidar.TimedLidarReadingFunc = func(ctx context.Context) (s.TimedLidarReadingResponse, error) {
			return s.TimedLidarReadingResponse{}, replaypcd.ErrEndOfDataset
		}

		injectMovementSensor.TimedMovementSensorReadingFunc = func(ctx context.Context) (s.TimedMovementSensorReadingResponse, error) {
			return movementSensorReading, nil
		}
		injectMovementSensor.PropertiesFunc = func() s.MovementSensorProperties {
			return s.MovementSensorProperties{
				IMUSupported:      true,
				OdometerSupported: true,
			}
		}
		config.MovementSensor = &injectMovementSensor

		countAddedLidarData := 0
		cf.AddLidarReadingFunc = func(ctx context.Context, timeout time.Duration,
			lidarName string, currentReading s.TimedLidarReadingResponse,
		) error {
			countAddedLidarData++
			return nil
		}

		countAddedIMUData := 0
		cf.AddIMUReadingFunc = func(ctx context.Context, timeout time.Duration,
			imuName string, currentReading s.TimedIMUReadingResponse,
		) error {
			countAddedIMUData++
			return nil
		}

		countAddedOdometerData := 0
		cf.AddOdometerReadingFunc = func(ctx context.Context, timeout time.Duration,
			odometerName string, currentReading s.TimedOdometerReadingResponse,
		) error {
			countAddedIMUData++
			return nil
		}

		endOfDataSetReached := config.StartOfflineSensorProcess(context.Background())
		test.That(t, endOfDataSetReached, test.ShouldBeTrue)
		test.That(t, countAddedLidarData, test.ShouldEqual, 0)
		test.That(t, countAddedIMUData, test.ShouldEqual, 0)
		test.That(t, countAddedOdometerData, test.ShouldEqual, 0)
	})

	t.Run("returns true when lidar reaches the end of the dataset and the optimization function fails", func(t *testing.T) {
		cf.RunFinalOptimizationFunc = func(context.Context, time.Duration) error {
			return errors.New("test error")
		}

		lidar, ms := s.FinishedReplayLidar, s.NoMovementSensor
		dataFrequencyHz := 0
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, ms), string(lidar), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor
		config.MovementSensor = nil

		endOfDataSetReached := config.StartOfflineSensorProcess(context.Background())
		test.That(t, endOfDataSetReached, test.ShouldBeTrue)
	})

	t.Run("successful data insertion", func(t *testing.T) {
		config.Lidar = &injectLidar
		cf.RunFinalOptimizationFunc = func(context.Context, time.Duration) error {
			return nil
		}

		cases := []struct {
			description             string
			imuEnabled              bool
			odometerEnabled         bool
			lidarReadingTimeAddedMs []int
			msReadingTimeAddedMs    []int
			expectedDataInsertions  []string
		}{
			{
				description:             "no movement sensor data is added if movement sensor is not enabled",
				odometerEnabled:         false,
				imuEnabled:              false,
				lidarReadingTimeAddedMs: []int{0, 2, 4, 6, 8},
				msReadingTimeAddedMs:    []int{1, 3, 5},
				expectedDataInsertions:  []string{"lidar: 0", "lidar: 2", "lidar: 4", "lidar: 6", "lidar: 8"},
			},
			{
				description:             "skip movement sensor data until first lidar data is inserted",
				imuEnabled:              true,
				odometerEnabled:         true,
				lidarReadingTimeAddedMs: []int{5, 7},
				msReadingTimeAddedMs:    []int{1, 2, 3, 4, 5, 6, 7, 8},
				expectedDataInsertions:  []string{"lidar: 5", "odometer: 5", "imu: 5", "odometer: 6", "imu: 6", "lidar: 7"},
			},
			{
				description:             "if imu data ends before lidar data ends, stop adding data once end of imu dataset is reached",
				imuEnabled:              true,
				odometerEnabled:         false,
				lidarReadingTimeAddedMs: []int{2, 4, 6, 8, 10, 12},
				msReadingTimeAddedMs:    []int{1, 3, 5},
				expectedDataInsertions:  []string{"lidar: 2", "imu: 3", "lidar: 4", "imu: 5"},
			},
			{
				description:             "if odometer data ends before lidar data ends, stop adding data once end of odometer dataset is reached",
				imuEnabled:              false,
				odometerEnabled:         true,
				lidarReadingTimeAddedMs: []int{2, 4, 6, 8, 10, 12},
				msReadingTimeAddedMs:    []int{1, 3, 5},
				expectedDataInsertions:  []string{"lidar: 2", "odometer: 3", "lidar: 4", "odometer: 5"},
			},
			{
				description:             "if movement sensor data ends before lidar data ends, stop adding data once end of movement sensor dataset is reached",
				imuEnabled:              true,
				odometerEnabled:         true,
				lidarReadingTimeAddedMs: []int{2, 4, 6, 8, 10, 12},
				msReadingTimeAddedMs:    []int{1, 3, 5},
				expectedDataInsertions:  []string{"lidar: 2", "odometer: 3", "imu: 3", "lidar: 4", "odometer: 5", "imu: 5"},
			},
			{
				description:             "if lidar data ends before imu data ends, stop adding data once end of lidar dataset is reached",
				imuEnabled:              true,
				odometerEnabled:         false,
				lidarReadingTimeAddedMs: []int{1, 3, 5},
				msReadingTimeAddedMs:    []int{2, 3, 4, 6, 8, 10, 12},
				expectedDataInsertions:  []string{"lidar: 1", "imu: 2", "lidar: 3", "imu: 3", "imu: 4", "lidar: 5"},
			},
			{
				description:             "if lidar data ends before odometer data ends, stop adding data once end of lidar dataset is reached",
				imuEnabled:              false,
				odometerEnabled:         true,
				lidarReadingTimeAddedMs: []int{1, 3, 5},
				msReadingTimeAddedMs:    []int{2, 3, 4, 6, 8, 10, 12},
				expectedDataInsertions:  []string{"lidar: 1", "odometer: 2", "lidar: 3", "odometer: 3", "odometer: 4", "lidar: 5"},
			},
			{
				description:             "if lidar data ends before movement sensor data ends, stop adding data once end of lidar dataset is reached",
				imuEnabled:              true,
				odometerEnabled:         true,
				lidarReadingTimeAddedMs: []int{1, 3, 5},
				msReadingTimeAddedMs:    []int{2, 3, 4, 6, 8, 10, 12},
				expectedDataInsertions:  []string{"lidar: 1", "odometer: 2", "imu: 2", "lidar: 3", "odometer: 3", "imu: 3", "odometer: 4", "imu: 4", "lidar: 5"},
			},
		}

		for _, tt := range cases {
			t.Run(tt.description, func(t *testing.T) {
				now := time.Now().UTC()

				if tt.imuEnabled || tt.odometerEnabled {
					config.MovementSensor = &injectMovementSensor
				} else {
					config.MovementSensor = nil
				}

				numLidarData := 0
				injectLidar.TimedLidarReadingFunc = func(ctx context.Context) (s.TimedLidarReadingResponse, error) {
					if numLidarData < len(tt.lidarReadingTimeAddedMs) {
						lidarReading.ReadingTime = now.Add(time.Duration(tt.lidarReadingTimeAddedMs[numLidarData]) * time.Millisecond)
						numLidarData++
						return lidarReading, nil
					}
					return s.TimedLidarReadingResponse{}, replaypcd.ErrEndOfDataset
				}

				numMovementSensorData := 0
				injectMovementSensor.TimedMovementSensorReadingFunc = func(ctx context.Context) (s.TimedMovementSensorReadingResponse, error) {
					var movementSensorReading s.TimedMovementSensorReadingResponse
					if numMovementSensorData < len(tt.msReadingTimeAddedMs) {
						if tt.odometerEnabled {
							movementSensorReading.TimedOdometerResponse = &odometerReading
							movementSensorReading.TimedOdometerResponse.ReadingTime = now.Add(time.Duration(tt.msReadingTimeAddedMs[numMovementSensorData]) * time.Millisecond)
						}
						if tt.imuEnabled {
							movementSensorReading.TimedIMUResponse = &imuReading
							movementSensorReading.TimedIMUResponse.ReadingTime = now.Add(time.Duration(tt.msReadingTimeAddedMs[numMovementSensorData]) * time.Millisecond)
						}
						numMovementSensorData++
						return movementSensorReading, nil
					}
					return movementSensorReading, replay.ErrEndOfDataset
				}
				injectMovementSensor.PropertiesFunc = func() s.MovementSensorProperties {
					return s.MovementSensorProperties{
						IMUSupported:      tt.imuEnabled,
						OdometerSupported: tt.odometerEnabled,
					}
				}

				actualDataInsertions := []string{}

				countAddedLidarData := 0
				cf.AddLidarReadingFunc = func(ctx context.Context, timeout time.Duration,
					lidarName string, currentReading s.TimedLidarReadingResponse,
				) error {
					actualDataInsertions = append(actualDataInsertions, "lidar: "+fmt.Sprint(tt.lidarReadingTimeAddedMs[numLidarData-1]))
					countAddedLidarData++
					return nil
				}

				countAddedIMUData := 0
				cf.AddIMUReadingFunc = func(ctx context.Context, timeout time.Duration,
					imuName string, currentReading s.TimedIMUReadingResponse,
				) error {
					actualDataInsertions = append(actualDataInsertions, "imu: "+fmt.Sprint(tt.msReadingTimeAddedMs[numMovementSensorData-1]))
					countAddedIMUData++
					return nil
				}

				countAddedOdometerData := 0
				cf.AddOdometerReadingFunc = func(ctx context.Context, timeout time.Duration,
					odometerName string, currentReading s.TimedOdometerReadingResponse,
				) error {
					actualDataInsertions = append(actualDataInsertions, "odometer: "+fmt.Sprint(tt.msReadingTimeAddedMs[numMovementSensorData-1]))
					countAddedOdometerData++
					return nil
				}

				countItemsInList := func(list []string, keyword string) int {
					counter := 0
					for _, item := range list {
						if strings.Contains(item, keyword) {
							counter++
						}
					}
					return counter
				}

				expectedCountAddedLidarData := countItemsInList(tt.expectedDataInsertions, "lidar")
				expectedCountAddedIMUData := countItemsInList(tt.expectedDataInsertions, "imu")
				expectedCountAddedOdometerData := countItemsInList(tt.expectedDataInsertions, "odometer")

				endOfDataSetReached := config.StartOfflineSensorProcess(context.Background())
				test.That(t, endOfDataSetReached, test.ShouldBeTrue)
				test.That(t, countAddedLidarData, test.ShouldEqual, expectedCountAddedLidarData)
				test.That(t, countAddedIMUData, test.ShouldEqual, expectedCountAddedIMUData)
				test.That(t, countAddedOdometerData, test.ShouldEqual, expectedCountAddedOdometerData)
				test.That(t, actualDataInsertions, test.ShouldResemble, tt.expectedDataInsertions)
			})
		}
	})
}

func TestTryAddLidarReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	reading := s.TimedLidarReadingResponse{
		Reading:     []byte("12345"),
		ReadingTime: time.Now().UTC(),
	}

	dataFrequencyHz := 5
	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}
	t.Run("when AddLidarReading blocks for more than the data rate and succeeds, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return nil
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading is slower than data rate and returns a lock error, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading blocks for more than the date rate "+
		"and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return errUnknown
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddLidarReading is faster than the date rate and succeeds, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			return nil
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.Lidar.DataFrequencyHz())
	})

	t.Run("when AddLidarReading is faster than the date rate "+
		"and returns lock error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.Lidar.DataFrequencyHz())
	})

	t.Run("when AddLidarReading is faster than date rate "+
		"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			return errUnknown
		}

		timeToSleep := config.tryAddLidarReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.Lidar.DataFrequencyHz())
	})
}

func TestAddLidarReadingInOnline(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	dataFrequencyHz := 5
	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}

	t.Run("returns error when lidar GetData returns error, doesn't try to add lidar data", func(t *testing.T) {
		invalidOnlineModeLidarTestHelper(context.Background(), t, cf, config, s.LidarWithErroringFunctions, 10)
	})

	t.Run("returns error when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		invalidOnlineModeLidarTestHelper(context.Background(), t, cf, config, s.InvalidReplayLidar, 10)
	})

	t.Run("online lidar adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeLidarTestHelper(context.Background(), t, config, cf, s.GoodLidar)
	})
}

func TestStartLidar(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	dataFrequencyHz := 5
	injectLidar := inject.TimedLidar{}
	injectLidar.NameFunc = func() string { return "good_lidar" }
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}

	t.Run("exits loop when the context was cancelled", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		lidar, imu := s.FinishedReplayLidar, s.NoMovementSensor
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		config.Lidar = replaySensor

		cancelFunc()

		config.StartLidar(cancelCtx)
	})
}

func TestTryAddIMUReading(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	imuReading := s.TimedIMUReadingResponse{
		LinearAcceleration: r3.Vector{X: 1, Y: 1, Z: 1},
		AngularVelocity:    spatialmath.AngularVelocity{X: 1, Y: 1, Z: 1},
		ReadingTime:        time.Now().UTC(),
	}

	reading := s.TimedMovementSensorReadingResponse{
		TimedIMUResponse: &imuReading,
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	injectImu := inject.TimedMovementSensor{}
	injectImu.NameFunc = func() string { return "good_imu" }
	injectImu.DataFrequencyHzFunc = func() int { return 20 }
	injectImu.PropertiesFunc = func() s.MovementSensorProperties {
		return s.MovementSensorProperties{
			IMUSupported: true,
		}
	}

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:          &injectLidar,
		MovementSensor: &injectImu,
		Timeout:        10 * time.Second,
	}

	t.Run("when AddIMUReading blocks for more than the date rate and succeeds, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return nil
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading is slower than date rate and returns a lock error, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading blocks for more than the date rate and returns an unexpected error, time to sleep is 0", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			time.Sleep(1 * time.Second)
			return errUnknown
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldEqual, 0)
	})

	t.Run("when AddIMUReading is faster than the date rate and succeeds, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			return nil
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
	})

	t.Run("when AddIMUReading is faster than the date rate and returns a lock error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			return cartofacade.ErrUnableToAcquireLock
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
	})

	t.Run("when AddIMUReading is faster than date rate "+
		"and returns an unexpected error, time to sleep is <= date rate", func(t *testing.T) {
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			return errUnknown
		}

		timeToSleep := config.tryAddMovementSensorReadingOnce(context.Background(), reading)
		test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
		test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, 1000/config.MovementSensor.DataFrequencyHz())
	})
}

func TestAddIMUReadingInOnline(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	injectImu := inject.TimedMovementSensor{}
	injectImu.NameFunc = func() string { return "good_imu" }
	injectImu.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       true,
		MovementSensor: &injectImu,
		Timeout:        10 * time.Second,
	}

	t.Run("returns error when IMU GetData returns error, doesn't try to add IMU data", func(t *testing.T) {
		invalidOnlineModeIMUTestHelper(context.Background(), t, cf, config, 10, s.IMUWithErroringFunctions, 10)
	})

	t.Run("returns error when replay sensor timestamp is invalid, doesn't try to add sensor data", func(t *testing.T) {
		invalidOnlineModeIMUTestHelper(context.Background(), t, cf, config, 10, s.InvalidReplayIMU, 10)
	})

	t.Run("online replay IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeIMUTestHelper(context.Background(), t, config, cf, s.ReplayIMU)
	})

	t.Run("online IMU adds sensor reading once and ignores errors", func(t *testing.T) {
		onlineModeIMUTestHelper(context.Background(), t, config, cf, s.GoodIMU)
	})
}

func TestStartIMU(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return 5 }

	injectImu := inject.TimedMovementSensor{}
	injectImu.NameFunc = func() string { return "good_imu" }
	injectImu.DataFrequencyHzFunc = func() int { return 20 }

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       injectLidar.DataFrequencyHzFunc() != 0,
		Lidar:          &injectLidar,
		MovementSensor: &injectImu,
		Timeout:        10 * time.Second,
	}

	t.Run("exits loop when the context was cancelled", func(t *testing.T) {
		cancelCtx, cancelFunc := context.WithCancel(context.Background())

		lidar, imu := s.NoLidar, s.FinishedReplayIMU
		replaySensor, err := s.NewMovementSensor(context.Background(), s.SetupDeps(lidar, imu), string(imu), 20, logger)
		test.That(t, err, test.ShouldBeNil)

		config.MovementSensor = replaySensor

		cancelFunc()

		config.StartMovementSensor(cancelCtx)
	})
}

func TestTryAddLidarReadingUntilSuccess(t *testing.T) {
	logger := logging.NewTestLogger(t)
	cf := cartofacade.Mock{}
	cf.RunFinalOptimizationFunc = func(context.Context, time.Duration) error {
		return nil
	}

	dataFrequencyHz := 0

	lidarReading := s.TimedLidarReadingResponse{
		Reading:     []byte("12345"),
		ReadingTime: time.Now().UTC(),
	}

	injectLidar := inject.TimedLidar{}
	injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:      logger,
		CartoFacade: &cf,
		IsOnline:    false,
		Lidar:       &injectLidar,
		Timeout:     10 * time.Second,
	}

	t.Run("replay lidar adds sensor data until success", func(t *testing.T) {
		lidar, imu := s.ReplayLidar, s.NoMovementSensor
		replaySensor, err := s.NewLidar(context.Background(), s.SetupDeps(lidar, imu), string(lidar), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		var calls []addLidarReadingArgs
		cf.AddLidarReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedLidarReadingResponse,
		) error {
			args := addLidarReadingArgs{
				timeout:        timeout,
				sensorName:     sensorName,
				currentReading: currentReading,
			}
			calls = append(calls, args)
			if len(calls) == 1 {
				return errUnknown
			}
			if len(calls) == 2 {
				return cartofacade.ErrUnableToAcquireLock
			}
			return nil
		}
		config.Lidar = replaySensor

		config.tryAddLidarReadingUntilSuccess(context.Background(), lidarReading)
		test.That(t, len(calls), test.ShouldEqual, 3)

		firstTimestamp := calls[0].currentReading.ReadingTime
		for i, call := range calls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, string(lidar))
			test.That(t, call.currentReading.Reading, test.ShouldResemble, lidarReading.Reading)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.currentReading.ReadingTime, test.ShouldEqual, firstTimestamp)
		}
	})
}

func TestTryAddIMUReadingUntilSuccess(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	cf := cartofacade.Mock{}

	dataFrequencyHz := 0
	injectImu := inject.TimedMovementSensor{}
	injectImu.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

	config := Config{
		Logger:         logger,
		CartoFacade:    &cf,
		IsOnline:       false,
		MovementSensor: &injectImu,
		Timeout:        10 * time.Second,
	}

	t.Run("replay IMU adds sensor data until success", func(t *testing.T) {
		lidar, imu := s.NoLidar, s.ReplayIMU
		replayIMU, err := s.NewMovementSensor(context.Background(), s.SetupDeps(lidar, imu), string(imu), dataFrequencyHz, logger)
		test.That(t, err, test.ShouldBeNil)

		injectLidar := inject.TimedLidar{}
		injectLidar.DataFrequencyHzFunc = func() int { return dataFrequencyHz }

		var calls []addIMUReadingArgs
		cf.AddIMUReadingFunc = func(
			ctx context.Context,
			timeout time.Duration,
			sensorName string,
			currentReading s.TimedIMUReadingResponse,
		) error {
			args := addIMUReadingArgs{
				timeout:        timeout,
				sensorName:     sensorName,
				currentReading: currentReading,
			}
			calls = append(calls, args)
			if len(calls) == 1 {
				return errUnknown
			}
			if len(calls) == 2 {
				return cartofacade.ErrUnableToAcquireLock
			}
			return nil
		}
		config.MovementSensor = replayIMU
		config.Lidar = &injectLidar

		config.tryAddMovementSensorReadingUntilSuccess(ctx, expectedMovementSensorReading)
		test.That(t, len(calls), test.ShouldEqual, 3)

		firstTimestamp := calls[0].currentReading.ReadingTime
		for i, call := range calls {
			t.Logf("call %d", i)
			test.That(t, call.sensorName, test.ShouldResemble, string(imu))
			test.That(t, call.currentReading.LinearAcceleration, test.ShouldResemble, expectedIMUReading.LinearAcceleration)
			test.That(t, call.currentReading.AngularVelocity, test.ShouldResemble, expectedIMUReading.AngularVelocity)
			test.That(t, call.timeout, test.ShouldEqual, config.Timeout)
			test.That(t, call.currentReading.ReadingTime, test.ShouldEqual, firstTimestamp)
		}
	})
}
