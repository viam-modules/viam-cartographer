package sensorprocess

import (
	"bytes"
	"context"
	"errors"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/utils/contextutils"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
)

func TestAddSensorReadingReplaySensor(t *testing.T) {
	logger := golog.NewTestLogger(t)
	buf := bytes.NewBuffer([]byte("12345"))
	cf := cartofacade.Mock{}
	cf.AddSensorReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading []byte,
		readingTimestamp time.Time,
	) error {
		return nil
	}
	// When addSensorReading returns successfully, no infinite loop
	params := Params{
		Ctx:               context.Background(),
		Logger:            logger,
		Cartofacade:       &cf,
		PrimarySensorName: "good_lidar",
		DataRateMs:        200,
		timeReq:           time.Now(),
		Timeout:           10 * time.Second,
		buf:               buf,
		failures:          make(map[time.Time]int),
	}
	AddSensorReadingFromReplaySensor(params)

	cf.AddSensorReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading []byte,
		readingTimestamp time.Time,
	) error {
		return errors.New("cant acquire lock")
	}

	// When addSensorReading is erroring and the context is cancelled, no infinite loop
	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	cancelFunc()

	params.Ctx = cancelCtx
	AddSensorReadingFromReplaySensor(params)

	// When addSensorReading fails a few times and then succeeds
	cancelCtx, cancelFunc = context.WithCancel(context.Background())
	params.Ctx = cancelCtx

	counter := 0
	var firstReading []byte
	var lastReading []byte

	cf.AddSensorReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading []byte,
		readingTimestamp time.Time,
	) error {
		if counter == 0 {
			firstReading = currentReading
		}
		if counter < 4 {
			counter++
			return errors.New("cant acquire lock")
		}
		lastReading = currentReading
		return nil
	}
	AddSensorReadingFromReplaySensor(params)
	// addSensorReading was called 4 times and the time it succeeded it was called with the same reading it was called with the first time
	test.That(t, counter, test.ShouldEqual, 4)
	test.That(t, firstReading, test.ShouldResemble, lastReading)
	cancelFunc()
}

func TestAddSensorReadingLiveReadings(t *testing.T) {
	logger := golog.NewTestLogger(t)
	cf := cartofacade.Mock{}
	buf := bytes.NewBuffer([]byte("12345"))
	cf.AddSensorReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading []byte,
		readingTimestamp time.Time,
	) error {
		time.Sleep(1 * time.Second)
		return nil
	}

	// When addSensorReading blocks for longer than the data rate
	params := Params{
		Ctx:               context.Background(),
		Logger:            logger,
		Cartofacade:       &cf,
		PrimarySensorName: "good_lidar",
		DataRateMs:        200,
		timeReq:           time.Now(),
		buf:               buf,
		Timeout:           10 * time.Second,
	}
	timeToSleep := AddSensorReadingFromLiveReadings(params)
	test.That(t, timeToSleep, test.ShouldEqual, 0)

	cf.AddSensorReadingFunc = func(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading []byte,
		readingTimestamp time.Time,
	) error {
		return nil
	}
	params.Cartofacade = &cf

	// When addSensorReading does not block for longer than the data rate
	dataRateMs := 200
	timeToSleep = AddSensorReadingFromLiveReadings(params)
	test.That(t, timeToSleep, test.ShouldBeGreaterThan, 0)
	test.That(t, timeToSleep, test.ShouldBeLessThanOrEqualTo, dataRateMs)
}

func TestAddSensorReading(t *testing.T) {
	logger := golog.NewTestLogger(t)

	sensors := []string{"good_lidar"}
	sensor, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
	test.That(t, err, test.ShouldBeNil)

	addSensorReadingFromReplaySensor := func(params Params) {
	}

	addSensorReadingFromLiveReadings := func(params Params) int {
		return 100
	}
	cf := cartofacade.Mock{}

	params := Params{
		Ctx:               context.Background(),
		Logger:            logger,
		Cartofacade:       &cf,
		PrimarySensorName: "replay_sensor",
		DataRateMs:        200,
		timeReq:           time.Now(),
		Lidar:             sensor,
		Timeout:           10 * time.Second,
	}

	// Live lidar
	params.addSensorReadingFromReplaySensor = addSensorReadingFromReplaySensor
	params.addSensorReadingFromLiveReadings = addSensorReadingFromLiveReadings
	err = AddSensorReading(params)
	test.That(t, err, test.ShouldBeNil)

	// Replay sensor with valid time
	sensors = []string{"replay_sensor"}
	replaySensor, err := lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
	test.That(t, err, test.ShouldBeNil)

	ctx, _ := contextutils.ContextWithMetadata(context.Background())
	params.Ctx = ctx
	params.Lidar = replaySensor
	params.PrimarySensorName = "replay_sensor"
	params.addSensorReadingFromReplaySensor = addSensorReadingFromReplaySensor
	params.addSensorReadingFromLiveReadings = addSensorReadingFromLiveReadings
	err = AddSensorReading(params)
	test.That(t, err, test.ShouldBeNil)

	// Replay sensor with invalid time
	sensors = []string{"invalid_replay_sensor"}
	replaySensor, err = lidar.New(testhelper.SetupDeps(sensors), sensors, 0)
	test.That(t, err, test.ShouldBeNil)

	ctx, _ = contextutils.ContextWithMetadata(context.Background())
	params.Ctx = ctx
	params.Lidar = replaySensor
	params.PrimarySensorName = "invalid_replay_sensor"
	params.addSensorReadingFromReplaySensor = addSensorReadingFromReplaySensor
	params.addSensorReadingFromLiveReadings = addSensorReadingFromLiveReadings
	err = AddSensorReading(params)
	test.That(t, err, test.ShouldBeError)
}

func TestStartBackgroundLogAggregator(t *testing.T) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	params := Params{
		Ctx:        cancelCtx,
		DataRateMs: 200,
		timeReq:    time.Now(),
		Timeout:    10 * time.Second,
		LogFreq:    10 * time.Millisecond,
		failures:   make(map[time.Time]int),
	}

	counter := 1
	logInfo := func(logData map[time.Time]int, logger golog.Logger) {
		counter++
		cancelFunc()
	}

	// test that callback function gets called properly
	startBackgroundLogAggregator(params, logInfo)
	test.That(t, counter, test.ShouldEqual, 2)
}

func TestIncrementLogCount(t *testing.T) {
	params := Params{
		failures: make(map[time.Time]int),
	}

	// when the time stamp isn't present in the map
	timestamp := time.Now()
	incrementLogCount(params, timestamp)
	test.That(t, params.failures[timestamp], test.ShouldEqual, 1)

	// when the time stamp is already present in the map
	incrementLogCount(params, timestamp)
	test.That(t, params.failures[timestamp], test.ShouldEqual, 2)
}
