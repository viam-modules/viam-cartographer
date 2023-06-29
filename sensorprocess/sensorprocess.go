package sensorprocess

import (
	"bytes"
	"context"
	"math"
	"time"

	"github.com/edaniels/golog"
	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/utils/contextutils"
)

// SensorProcess polls the lidar to get the next sensor reading and adds it to the mapBuilder.
func SensorProcess(
	ctx context.Context,
	lidar lidar.Lidar,
	timeout time.Duration,
	addSensorReading func(
		ctx context.Context,
		lidar lidar.Lidar,
		timeout time.Duration,
	),
) {
	/*
		TODO: The tracing / telemetry should output aggregates & not be 1 log line per
		failure to acquire the lock as that would introduce performance issues.
	*/
	for {
		select {
		case <-ctx.Done():
			return
		default:
			addSensorReading(ctx, lidar, timeout)
		}
	}
}

//nolint:unused
func addSensorReading(
	ctx context.Context,
	lidar lidar.Lidar,
	timeout time.Duration,
	addSensorReadingReplaySensor func(context.Context, time.Time, *bytes.Buffer, time.Duration) error,
	addSensorReadingLidar func(context.Context, time.Time, *bytes.Buffer, time.Duration) int,
) error {
	reading, err := lidar.GetData(ctx)
	if err != nil {
		return err
	}

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(reading, buf, 0)
	if err != nil {
		return err
	}

	timeReq := time.Now()
	ctx, md := contextutils.ContextWithMetadata(ctx)
	timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]

	if ok {
		timeReq, err := time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
		if err != nil {
			return err
		}

		err = addSensorReadingReplaySensor(ctx, timeReq, buf, timeout)
		if err != nil {
			return err
		}
	} else {
		timeToSleep := addSensorReadingLidar(ctx, timeReq, buf, timeout)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	}

	return nil
}

func addSensorReadingReplaySensor(
	ctx context.Context,
	logger golog.Logger,
	cartofacade cartofacade.Interface,
	primarySensorName string,
	dataRateMs int,
	timeReq time.Time,
	buf *bytes.Buffer,
	timeout time.Duration,
) {
	err := cartofacade.AddSensorReading(ctx, timeout, primarySensorName, buf.Bytes(), timeReq)
	/*
		while add sensor reading fails, keep trying to add the same reading - in offline mode
		we want to process each reading so if we cannot acquire the lock we should try again
	*/
	for err != nil {
		select {
		case <-ctx.Done():
			return
		default:
			logger.Warnf("Unable to acquire the lock for reading from %v. Trying again", timeReq)
			err = cartofacade.AddSensorReading(ctx, timeout, primarySensorName, buf.Bytes(), timeReq)
		}
	}
}

func addSensorReadingLidar(
	ctx context.Context,
	logger golog.Logger,
	cartofacade cartofacade.Interface,
	primarySensorName string,
	dataRateMs int,
	timeReq time.Time,
	buf *bytes.Buffer,
	timeout time.Duration,
) int {
	startTime := time.Now()
	err := cartofacade.AddSensorReading(ctx, timeout, primarySensorName, buf.Bytes(), timeReq)
	if err != nil {
		logger.Warnf("Unable to acquire the lock. Not processing reading from %v", timeReq)
	}
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(dataRateMs-timeElapsedMs)))
}
