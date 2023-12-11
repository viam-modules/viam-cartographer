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
