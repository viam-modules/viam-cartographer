package sensorprocess

import (
	"bytes"
	"context"
	"errors"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"go.viam.com/test"
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
	addSensorReadingReplaySensor(context.Background(), logger, &cf, "good_lidar", 200, time.Now(), buf, 10*time.Second)

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
	addSensorReadingReplaySensor(cancelCtx, logger, &cf, "good_lidar", 200, time.Now(), buf, 10*time.Second)
}

func TestAddSensorReadingOnline(t *testing.T) {
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
	timeToSleep := addSensorReadingLidar(context.Background(), logger, &cf, "good_lidar", 200, time.Now(), buf, 10*time.Second)
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

	// When addSensorReading does not block for longer than the data rate
	timeToSleep = addSensorReadingLidar(context.Background(), logger, &cf, "good_lidar", 200, time.Now(), buf, 10*time.Second)
	test.That(t, timeToSleep, test.ShouldNotEqual, 0)
}
