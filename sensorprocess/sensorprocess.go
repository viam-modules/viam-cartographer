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

type SensorProcessParams struct {
	Ctx               context.Context
	Cartofacade       cartofacade.Interface
	Lidar             lidar.Lidar
	PrimarySensorName string
	DataRateMs        int
	Timeout           time.Duration
	Logger            golog.Logger

	timeReq time.Time
	buf     *bytes.Buffer
}

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
func AddSensorReading(
	params SensorProcessParams,
	addSensorReadingFromReplaySensor func(SensorProcessParams),
	addSensorReadingFromLiveReadings func(SensorProcessParams) int,
) error {
	reading, err := params.Lidar.GetData(params.Ctx)
	if err != nil {
		return err
	}

	buf := new(bytes.Buffer)
	err = pointcloud.ToPCD(reading, buf, 0)
	if err != nil {
		return err
	}
	params.buf = buf

	timeReq := time.Now()
	ctx, md := contextutils.ContextWithMetadata(params.Ctx)
	timeRequestedMetadata, ok := md[contextutils.TimeRequestedMetadataKey]
	params.Ctx = ctx

	if ok {
		timeReq, err = time.Parse(time.RFC3339Nano, timeRequestedMetadata[0])
		if err != nil {
			return err
		}
		params.timeReq = timeReq
		addSensorReadingFromReplaySensor(params)
	} else {
		params.timeReq = timeReq
		timeToSleep := addSensorReadingFromLiveReadings(params)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	}

	return nil
}

func AddSensorReadingFromReplaySensor(params SensorProcessParams) {
	err := params.Cartofacade.AddSensorReading(params.Ctx, params.Timeout, params.PrimarySensorName, params.buf.Bytes(), params.timeReq)
	/*
		while add sensor reading fails, keep trying to add the same reading - in offline mode
		we want to process each reading so if we cannot acquire the lock we should try again
	*/
	for err != nil {
		select {
		case <-params.Ctx.Done():
			return
		default:
			params.Logger.Warnf("Unable to acquire the lock for reading from %v. Trying again", params.timeReq)
			err = params.Cartofacade.AddSensorReading(params.Ctx, params.Timeout, params.PrimarySensorName, params.buf.Bytes(), params.timeReq)
		}
	}
}

func AddSensorReadingFromLiveReadings(params SensorProcessParams) int {
	startTime := time.Now()
	err := params.Cartofacade.AddSensorReading(params.Ctx, params.Timeout, params.PrimarySensorName, params.buf.Bytes(), params.timeReq)
	if err != nil {
		params.Logger.Warnf("Unable to acquire the lock. Not processing reading from %v", params.timeReq)
	}
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(params.DataRateMs-timeElapsedMs)))
}
