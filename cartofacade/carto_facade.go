// Package cartofacade contains the api to call into CGO
package cartofacade

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"
)

// RequestType defines the carto C API call that is being made.
type RequestType int64

const (
	// Initialize represents the viam_carto_init call in c.
	Initialize RequestType = iota
	// Start represents the viam_carto_start call in c.
	Start
	// Stop represents the viam_carto_stop call in c.
	Stop
	// Terminate represents the viam_carto_terminate in c.
	Terminate
	// AddSensorReading represents the viam_carto_add_sensor_reading in c.
	AddSensorReading
	// Position represents the viam_carto_get_position call in c.
	Position
	// InternalState represents the viam_carto_get_internal_state call in c.
	InternalState
	// PointCloudMap represents the viam_carto_get_point_cloud_map in c.
	PointCloudMap
)

// RequestParamType defines the type being provided as input to the work.
type RequestParamType int64

const (
	// Sensor represents a sensor name input into c funcs.
	Sensor RequestParamType = iota
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
CartoFacade exists to ensure that only one go routine is calling into the CGO api at a time to ensure the
go runtime doesn't spawn multiple OS threads, which would harm performance.
*/
type CartoFacade struct {
	RequestChan     chan Request
	CartoLib        CartoLibInterface
	Carto           CartoInterface
	CartoConfig     CartoConfig
	CartoAlgoConfig CartoAlgoConfig
}

// RequestInterface defines the functionality of a WorkItem.
// It should not be used outside of this package but needs to be public for testing purposes.
type RequestInterface interface {
	DoWork(q *CartoFacade) (interface{}, error)
}

// Request defines all of the necessary pieces to call into the CGo API.
type Request struct {
	ResponseChan  chan Response
	requestType   RequestType
	requestParams map[RequestParamType]interface{}
}

// New instantiates the Cartofacade struct which limits calls into C.
func New(cartoLib CartoLibInterface, cartoCfg CartoConfig, cartoAlgoCfg CartoAlgoConfig) CartoFacade {
	return CartoFacade{
		RequestChan:     make(chan Request),
		Carto:           &Carto{},
		CartoLib:        cartoLib,
		CartoConfig:     cartoCfg,
		CartoAlgoConfig: cartoAlgoCfg,
	}
}

// DoWork provides the logic to call the correct cgo functions with the correct input.
// It should not be called outside of this package but needs to be public for testing purposes.
func (r *Request) DoWork(
	cf *CartoFacade,
) (interface{}, error) {
	switch r.requestType {
	case Initialize:
		return NewCarto(cf.CartoConfig, cf.CartoAlgoConfig, cf.CartoLib)
	case Start:
		return nil, cf.Carto.start()
	case Stop:
		return nil, cf.Carto.stop()
	case Terminate:
		return nil, cf.Carto.terminate()
	case AddSensorReading:
		sensor, ok := r.requestParams[Sensor].(string)
		if !ok {
			return nil, errors.New("could not cast inputted sensor name to string")
		}

		reading, ok := r.requestParams[Reading].([]byte)
		if !ok {
			return nil, errors.New("could not cast inputted byte to byte slice")
		}

		timestamp, ok := r.requestParams[Timestamp].(time.Time)
		if !ok {
			return nil, errors.New("could not cast inputted timestamp to times.Time")
		}

		return nil, cf.Carto.addSensorReading(sensor, reading, timestamp)
	case Position:
		return cf.Carto.getPosition()
	case InternalState:
		return cf.Carto.getInternalState()
	case PointCloudMap:
		return cf.Carto.getPointCloudMap()
	}
	return nil, fmt.Errorf("no worktype found for: %v", r.requestType)
}

// Request wraps calls into C. This function requires the caller to know which RequestTypes requires casting to which response values.
func (cf *CartoFacade) Request(
	ctxParent context.Context,
	requestType RequestType,
	inputs map[RequestParamType]interface{},
	timeout time.Duration,
) (interface{}, error) {
	ctx, cancel := context.WithTimeout(ctxParent, timeout)
	defer cancel()

	req := Request{
		ResponseChan:  make(chan Response, 1),
		requestType:   requestType,
		requestParams: inputs,
	}

	// wait until work can call into C (and timeout if needed)
	select {
	case cf.RequestChan <- req:
		select {
		case response := <-req.ResponseChan:
			return response.result, response.err
		case <-ctx.Done():
			return nil, errors.New("timeout has occurred while trying to read request from cartofacade")
		}
	case <-ctx.Done():
		return nil, errors.New("timeout has occurred while trying to write request to cartofacade. Did you forget to call cartoFacade.Start()?")
	}
}

// Start starts the background goroutine that is responsible for ensuring only one call into C is being made at a time.
func (cf *CartoFacade) Start(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup) {
	activeBackgroundWorkers.Add(1)
	go func() {
		defer activeBackgroundWorkers.Done()

		for {
			select {
			case <-ctx.Done():
				return
			case workToDo := <-cf.RequestChan:
				result, err := workToDo.DoWork(cf)
				workToDo.ResponseChan <- Response{result: result, err: err}
			}
		}
	}()
}
