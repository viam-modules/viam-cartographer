// Package cartofacade contains the api to call into CGO
package cartofacade

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"
)

var emptyRequestParams = map[RequestParamType]interface{}{}

// Initialize calls into the cartofacade C code.
func (cf *CartoFacade) Initialize(ctx context.Context, timeout time.Duration, activeBackgroundWorkers *sync.WaitGroup) error {
	cf.start(ctx, activeBackgroundWorkers)
	untyped, err := cf.request(ctx, initialize, emptyRequestParams, timeout)
	if err != nil {
		return err
	}

	carto, ok := untyped.(Carto)
	if !ok {
		return errors.New("unable to cast response from cartofacade to a carto struct")
	}

	cf.carto = &carto

	return nil
}

// Start calls into the cartofacade C code.
func (cf *CartoFacade) Start(ctx context.Context, timeout time.Duration) error {
	_, err := cf.request(ctx, start, emptyRequestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// Stop calls into the cartofacade C code.
func (cf *CartoFacade) Stop(ctx context.Context, timeout time.Duration) error {
	_, err := cf.request(ctx, stop, emptyRequestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// Terminate calls into the cartofacade C code.
func (cf *CartoFacade) Terminate(ctx context.Context, timeout time.Duration) error {
	_, err := cf.request(ctx, terminate, emptyRequestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// AddSensorReading calls into the cartofacade C code.
func (cf *CartoFacade) AddSensorReading(
	ctx context.Context,
	timeout time.Duration,
	sensorName string,
	currentReading []byte,
	readingTimestamp time.Time,
) error {
	requestParams := map[RequestParamType]interface{}{
		sensor:    sensorName,
		reading:   currentReading,
		timestamp: readingTimestamp,
	}

	_, err := cf.request(ctx, addSensorReading, requestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// GetPosition calls into the cartofacade C code.
func (cf *CartoFacade) GetPosition(ctx context.Context, timeout time.Duration) (GetPosition, error) {
	untyped, err := cf.request(ctx, position, emptyRequestParams, timeout)
	if err != nil {
		return GetPosition{}, err
	}

	pos, ok := untyped.(GetPosition)
	if !ok {
		return GetPosition{}, errors.New("unable to cast response from cartofacade to a position info struct")
	}

	return pos, nil
}

// GetInternalState calls into the cartofacade C code.
func (cf *CartoFacade) GetInternalState(ctx context.Context, timeout time.Duration) ([]byte, error) {
	untyped, err := cf.request(ctx, internalState, emptyRequestParams, timeout)
	if err != nil {
		return []byte{}, err
	}

	internalState, ok := untyped.([]byte)
	if !ok {
		return []byte{}, errors.New("unable to cast response from cartofacade to a byte slice")
	}

	return internalState, nil
}

// GetPointCloudMap calls into the cartofacade C code.
func (cf *CartoFacade) GetPointCloudMap(ctx context.Context, timeout time.Duration) ([]byte, error) {
	untyped, err := cf.request(ctx, pointCloudMap, emptyRequestParams, timeout)
	if err != nil {
		return []byte{}, err
	}

	pointCloud, ok := untyped.([]byte)
	if !ok {
		return []byte{}, errors.New("unable to cast response from cartofacade to a byte slice")
	}

	return pointCloud, nil
}

// RequestType defines the carto C API call that is being made.
type RequestType int64

const (
	// initialize represents the viam_carto_init call in c.
	initialize RequestType = iota
	// start represents the viam_carto_start call in c.
	start
	// stop represents the viam_carto_stop call in c.
	stop
	// terminate represents the viam_carto_terminate in c.
	terminate
	// addSensorReading represents the viam_carto_add_sensor_reading in c.
	addSensorReading
	// position represents the viam_carto_get_position call in c.
	position
	// internalState represents the viam_carto_get_internal_state call in c.
	internalState
	// pointCloudMap represents the viam_carto_get_point_cloud_map in c.
	pointCloudMap
)

// RequestParamType defines the type being provided as input to the work.
type RequestParamType int64

const (
	// sensor represents a sensor name input into c funcs.
	sensor RequestParamType = iota
	// reading represents a lidar reading input into c funcs.
	reading
	// timestamp represents the timestamp input into c funcs.
	timestamp
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
	cartoLib        CartoLibInterface
	carto           CartoInterface
	cartoConfig     CartoConfig
	cartoAlgoConfig CartoAlgoConfig
	requestChan     chan Request
}

// RequestInterface defines the functionality of a Request.
// It should not be used outside of this package but needs to be public for testing purposes.
type RequestInterface interface {
	doWork(q *CartoFacade) (interface{}, error)
}

// Interface defines the functionality of a CartoFacade instance.
// It should not be used outside of this package but needs to be public for testing purposes.
type Interface interface {
	request(
		ctxParent context.Context,
		requestType RequestType,
		inputs map[RequestParamType]interface{}, timeout time.Duration,
	) (interface{}, error)
	start(
		ctx context.Context,
		activeBackgroundWorkers *sync.WaitGroup,
	)

	Initialize(
		ctx context.Context,
		timeout time.Duration,
		activeBackgroundWorkers *sync.WaitGroup,
	) error
	Start(
		ctx context.Context,
		timeout time.Duration,
	) error
	Stop(
		ctx context.Context,
		timeout time.Duration,
	) error
	Terminate(
		ctx context.Context,
		timeout time.Duration,
	) error
	AddSensorReading(
		ctx context.Context,
		timeout time.Duration,
		sensorName string,
		currentReading []byte,
		readingTimestamp time.Time,
	) error
	GetPosition(
		ctx context.Context,
		timeout time.Duration,
	) (GetPosition, error)
	GetInternalState(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error)
	GetPointCloudMap(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error)
}

// Request defines all of the necessary pieces to call into the CGo API.
type Request struct {
	responseChan  chan Response
	requestType   RequestType
	requestParams map[RequestParamType]interface{}
}

// New instantiates the Cartofacade struct which limits calls into C.
func New(cartoLib CartoLibInterface, cartoCfg CartoConfig, cartoAlgoCfg CartoAlgoConfig) CartoFacade {
	return CartoFacade{
		carto:           &Carto{},
		cartoLib:        cartoLib,
		cartoConfig:     cartoCfg,
		cartoAlgoConfig: cartoAlgoCfg,
		requestChan:     make(chan Request),
	}
}

// DoWork provides the logic to call the correct cgo functions with the correct input.
// It should not be called outside of this package but needs to be public for testing purposes.
func (r *Request) doWork(
	cf *CartoFacade,
) (interface{}, error) {
	switch r.requestType {
	case initialize:
		return NewCarto(cf.cartoConfig, cf.cartoAlgoConfig, cf.cartoLib)
	case start:
		return nil, cf.carto.start()
	case stop:
		return nil, cf.carto.stop()
	case terminate:
		return nil, cf.carto.terminate()
	case addSensorReading:
		sensor, ok := r.requestParams[sensor].(string)
		if !ok {
			return nil, errors.New("could not cast inputted sensor name to string")
		}

		reading, ok := r.requestParams[reading].([]byte)
		if !ok {
			return nil, errors.New("could not cast inputted byte to byte slice")
		}

		timestamp, ok := r.requestParams[timestamp].(time.Time)
		if !ok {
			return nil, errors.New("could not cast inputted timestamp to times.Time")
		}

		return nil, cf.carto.addSensorReading(sensor, reading, timestamp)
	case position:
		return cf.carto.getPosition()
	case internalState:
		return cf.carto.getInternalState()
	case pointCloudMap:
		return cf.carto.getPointCloudMap()
	}
	return nil, fmt.Errorf("no worktype found for: %v", r.requestType)
}

// request wraps calls into C. This function requires the caller to know which RequestTypes requires casting to which response values.
func (cf *CartoFacade) request(
	ctxParent context.Context,
	requestType RequestType,
	inputs map[RequestParamType]interface{},
	timeout time.Duration,
) (interface{}, error) {
	ctx, cancel := context.WithTimeout(ctxParent, timeout)
	defer cancel()

	req := Request{
		responseChan:  make(chan Response, 1),
		requestType:   requestType,
		requestParams: inputs,
	}

	// wait until work can call into C (and timeout if needed)
	select {
	case cf.requestChan <- req:
		select {
		case response := <-req.responseChan:
			return response.result, response.err
		case <-ctx.Done():
			return nil, errors.New("timeout has occurred while trying to read request from cartofacade")
		}
	case <-ctx.Done():
		return nil, errors.New("timeout has occurred while trying to write request to cartofacade. Did you forget to call Start()?")
	}
}

// start starts the background goroutine that is responsible for ensuring only one call into C is being made at a time.
func (cf *CartoFacade) start(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup) {
	activeBackgroundWorkers.Add(1)
	go func() {
		defer activeBackgroundWorkers.Done()

		for {
			select {
			case <-ctx.Done():
				return
			case workToDo := <-cf.requestChan:
				result, err := workToDo.doWork(cf)
				workToDo.responseChan <- Response{result: result, err: err}
			}
		}
	}()
}
