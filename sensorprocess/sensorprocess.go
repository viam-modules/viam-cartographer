// Package sensorprocess contains the logic to add lidar or replay sensor readings to cartographer's mapbuilder
package sensorprocess

import (
	"bytes"
	"context"
	"math"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/utils/contextutils"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
)

// Params holds params needed throughout the process of adding a sensor reading to the mapbuilder.
type Params struct {
	Ctx               context.Context
	Cartofacade       cartofacade.Interface
	Lidar             lidar.Lidar
	PrimarySensorName string
	DataRateMs        int
	Timeout           time.Duration
	Logger            golog.Logger

	timeReq                          time.Time
	buf                              *bytes.Buffer
	addSensorReadingFromReplaySensor func(Params)
	addSensorReadingFromLiveReadings func(Params) int
}

// SensorProcess polls the lidar to get the next sensor reading and adds it to the mapBuilder.
func SensorProcess(
	params Params,
	addSensorReading func(
		params Params,
	),
) {
	/*
		TODO: The tracing / telemetry should output aggregates & not be 1 log line per
		failure to acquire the lock as that would introduce performance issues.
	*/
	for {
		select {
		case <-params.Ctx.Done():
			return
		default:
			addSensorReading(params)
		}
	}
}

// AddSensorReading adds a lidar reading to the mapbuilder.
func AddSensorReading(
	params Params,
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
		params.addSensorReadingFromReplaySensor(params)
	} else {
		params.timeReq = timeReq
		timeToSleep := params.addSensorReadingFromLiveReadings(params)
		time.Sleep(time.Duration(timeToSleep) * time.Millisecond)
	}

	return nil
}

// AddSensorReadingFromReplaySensor adds a reading from a replay sensor to the map builder.
func AddSensorReadingFromReplaySensor(params Params) {
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

// AddSensorReadingFromLiveReadings adds a reading from a live lidar to the map builder.
func AddSensorReadingFromLiveReadings(params Params) int {
	startTime := time.Now()
	err := params.Cartofacade.AddSensorReading(params.Ctx, params.Timeout, params.PrimarySensorName, params.buf.Bytes(), params.timeReq)
	if err != nil {
		params.Logger.Warnf("Unable to acquire the lock. Not processing reading from %v", params.timeReq)
	}
	timeElapsedMs := int(time.Since(startTime).Milliseconds())
	return int(math.Max(0, float64(params.DataRateMs-timeElapsedMs)))
}
