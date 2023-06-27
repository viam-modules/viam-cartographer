// Package cartofacade contains the api to call into CGO
package cartofacade

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	cgoApi "github.com/viamrobotics/viam-cartographer/cartofacade/internal/capi"
)

// WorkType defines the grpc call that is being made.
type WorkType int64

const (
	// Initialize can be used to represent the viam_carto_init call in c.
	Initialize WorkType = iota
	// Start can be used to represent the viam_carto_start call in c.
	Start
	// Stop can be used to represent the viam_carto_stop call in c.
	Stop
	// Terminate can be used to represent the viam_carto_terminate in c.
	Terminate
	// AddSensorReading can be used to represent the viam_carto_add_sensor_reading in c.
	AddSensorReading
	// Position can be used to represent the viam_carto_get_position call in c.
	Position
	// InternalState can be used to represent the viam_carto_get_internal_state call in c.
	InternalState
	// PointCloudMap can be used to represent the viam_carto_get_point_cloud_map in c.
	PointCloudMap
)

// InputType defines the type being provided as input to the work.
type InputType int64

const (
	// Sensor represents a sensor name input into c funcs.
	Sensor InputType = iota
	// Reading represents a lidar reading input into c funcs.
	Reading
	// Timestamp represents the timestamp input into c funcs.
	Timestamp
)

// Response defines the result of one piece of work that can be put on the result channel.
type Response struct {
	result interface{}
	err    error
}

/*
	CartoFacade represents a queue to consume work from and enforce one call into C at a time.

It exists to ensure that only one go routine is calling into the CGO api at a time to ensure the
go runtime doesn't spawn multiple OS threads, which would harm performance
*/
type CartoFacade struct {
	WorkChannel     chan WorkItem
	CartoLib        *cgoApi.CartoLibInterface
	Carto           cgoApi.CartoInterface
	CartoConfig     cgoApi.CartoConfig
	CartoAlgoConfig cgoApi.CartoAlgoConfig
}

// WorkItemInterface defines one piece of work that can be put on the queue.
// It should not be used outside of this package but needs to be public for testing purposes
type WorkItemInterface interface {
	DoWork(q *CartoFacade) (interface{}, error)
}

// WorkItem defines one piece of work that can be put on the queue.
type WorkItem struct {
	Result   chan Response
	workType WorkType
	inputs   map[InputType]interface{}
}

// New instantiates the Cartofacade struct which holds the queue that limits calls into C.
func New(cartoLib cgoApi.CartoLibInterface, cartoCfg cgoApi.CartoConfig, cartoAlgoCfg cgoApi.CartoAlgoConfig) CartoFacade {
	return CartoFacade{
		WorkChannel:     make(chan WorkItem),
		Carto:           &cgoApi.Carto{},
		CartoLib:        &cartoLib,
		CartoConfig:     cartoCfg,
		CartoAlgoConfig: cartoAlgoCfg,
	}
}

// DoWork provides the logic to call the correct cgo functions with the correct input.
// It should not be called outside of this package but needs to be public for testing purposes
func (w *WorkItem) DoWork(
	cf *CartoFacade,
) (interface{}, error) {
	switch w.workType {
	case Initialize:
		return cgoApi.New(cf.CartoConfig, cf.CartoAlgoConfig, *cf.CartoLib)
	case Start:
		return nil, cf.Carto.Start()
	case Stop:
		return nil, cf.Carto.Stop()
	case Terminate:
		return nil, cf.Carto.Terminate()
	case AddSensorReading:
		sensor, ok := w.inputs[Sensor].(string)
		if !ok {
			return nil, errors.New("could not cast inputted sensor name to string")
		}

		reading, ok := w.inputs[Reading].([]byte)
		if !ok {
			return nil, errors.New("could not cast inputted byte to byte slice")
		}

		timestamp, ok := w.inputs[Timestamp].(time.Time)
		if !ok {
			return nil, errors.New("could not cast inputted timestamp to times.Time")
		}

		return nil, cf.Carto.AddSensorReading(sensor, reading, timestamp)
	case Position:
		return cf.Carto.GetPosition()
	case InternalState:
		return cf.Carto.GetInternalState()
	case PointCloudMap:
		return cf.Carto.GetPointCloudMap()
	}
	return nil, fmt.Errorf("no worktype found for: %v", w.workType)
}

// Request puts incoming requests on the queue and consumes from queue.
func (cf *CartoFacade) Request(ctxParent context.Context, workType WorkType, inputs map[InputType]interface{}, timeout time.Duration) (interface{}, error) {
	ctx, cancel := context.WithTimeout(ctxParent, timeout)
	defer cancel()

	work := WorkItem{
		Result:   make(chan Response, 1),
		workType: workType,
		inputs:   inputs,
	}

	// wait until work can get put on the queue (and timeout if needed)
	select {
	case cf.WorkChannel <- work:
		select {
		case response := <-work.Result:
			return response.result, response.err
		case <-ctx.Done():
			return nil, errors.New("timeout has occurred while trying to read request from cartofacade")
		}
	case <-ctx.Done():
		return nil, errors.New("timeout has occurred while trying to write request to cartofacade. Did you start the background worker?")
	}
}

// Start starts the background goroutine that is responsible for putting work
// onto the queue and consuming from the queue.
func (cf *CartoFacade) Start(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup) {
	activeBackgroundWorkers.Add(1)
	go func() {
		defer activeBackgroundWorkers.Done()

		for {
			select {
			case <-ctx.Done():
				return
			case workToDo := <-cf.WorkChannel:
				result, err := workToDo.DoWork(cf)
				if err != nil {
					workToDo.Result <- Response{result: result, err: err}
				} else {
					workToDo.Result <- Response{result: result, err: err}
				}
			}
		}
	}()
}
