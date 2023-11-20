// Package cartofacade contains the api to call into CGO
package cartofacade

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	"go.uber.org/multierr"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

var emptyRequestParams = map[RequestParamType]interface{}{}

// ErrUnableToAcquireLock is the error returned from AddLidarReading and/or AddIMUReading when lock can't be acquired.
var ErrUnableToAcquireLock = errors.New("VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK")

// Initialize calls into the cartofacade C code.
func (cf *CartoFacade) Initialize(ctx context.Context, timeout time.Duration, activeBackgroundWorkers *sync.WaitGroup) (SlamMode, error) {
	cf.startCGoroutine(ctx, activeBackgroundWorkers)
	untyped, err := cf.request(ctx, initialize, emptyRequestParams, timeout)
	if err != nil {
		return UnknownMode, err
	}

	carto, ok := untyped.(Carto)
	if !ok {
		return UnknownMode, errors.New("unable to cast response from cartofacade to a carto struct")
	}

	cf.carto = &carto

	return carto.SlamMode, nil
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

// AddLidarReading calls into the cartofacade C code.
func (cf *CartoFacade) AddLidarReading(
	ctx context.Context,
	timeout time.Duration,
	lidarName string,
	currentReading s.TimedLidarReadingResponse,
) error {
	requestParams := map[RequestParamType]interface{}{
		sensor:  lidarName,
		reading: currentReading,
	}

	_, err := cf.request(ctx, addLidarReading, requestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// AddIMUReading calls into the cartofacade C code.
func (cf *CartoFacade) AddIMUReading(
	ctx context.Context,
	timeout time.Duration,
	imuName string,
	currentReading s.TimedIMUReadingResponse,
) error {
	requestParams := map[RequestParamType]interface{}{
		sensor:  imuName,
		reading: currentReading,
	}

	_, err := cf.request(ctx, addIMUReading, requestParams, timeout)
	if err != nil {
		return err
	}

	return nil
}

// Position calls into the cartofacade C code.
func (cf *CartoFacade) Position(ctx context.Context, timeout time.Duration) (Position, error) {
	untyped, err := cf.request(ctx, position, emptyRequestParams, timeout)
	if err != nil {
		return Position{}, err
	}

	pos, ok := untyped.(Position)
	if !ok {
		return Position{}, errors.New("unable to cast response from cartofacade to a position info struct")
	}

	return pos, nil
}

// InternalState calls into the cartofacade C code.
func (cf *CartoFacade) InternalState(ctx context.Context, timeout time.Duration) ([]byte, error) {
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

// PointCloudMap calls into the cartofacade C code.
func (cf *CartoFacade) PointCloudMap(ctx context.Context, timeout time.Duration) ([]byte, error) {
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

// RunFinalOptimization calls into the cartofacade C code.
func (cf *CartoFacade) RunFinalOptimization(ctx context.Context, timeout time.Duration) error {
	_, err := cf.request(ctx, runFinalOptimization, emptyRequestParams, timeout)
	if err != nil {
		return err
	}

	return nil
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
	// addLidarReading represents the viam_carto_add_lidar_reading in c.
	addLidarReading
	// addIMUReading represents the viam_carto_add_imu_reading in c.
	addIMUReading
	// position represents the viam_carto_get_position call in c.
	position
	// internalState represents the viam_carto_get_internal_state call in c.
	internalState
	// pointCloudMap represents the viam_carto_get_point_cloud_map in c.
	pointCloudMap
	// runFinalOptimization represents viam_carto_run_final_optimization.
	runFinalOptimization
)

// RequestParamType defines the type being provided as input to the work.
type RequestParamType int64

const (
	// sensor represents a sensor name input into c funcs.
	sensor RequestParamType = iota
	// reading represents a sensor reading input into c funcs.
	reading
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
	startCGoroutine(
		ctx context.Context,
		activeBackgroundWorkers *sync.WaitGroup,
	)

	Initialize(
		ctx context.Context,
		timeout time.Duration,
		activeBackgroundWorkers *sync.WaitGroup,
	) (SlamMode, error)
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
	AddLidarReading(
		ctx context.Context,
		timeout time.Duration,
		lidarName string,
		currentReading s.TimedLidarReadingResponse,
	) error
	AddIMUReading(
		ctx context.Context,
		timeout time.Duration,
		imuName string,
		currentReading s.TimedIMUReadingResponse,
	) error
	Position(
		ctx context.Context,
		timeout time.Duration,
	) (Position, error)
	InternalState(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error)
	PointCloudMap(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error)
	RunFinalOptimization(
		ctx context.Context,
		timeout time.Duration,
	) error
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

// doWork provides the logic to call the correct cgo functions with the correct input.
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
	case addLidarReading:
		lidar, ok := r.requestParams[sensor].(string)
		if !ok {
			return nil, errors.New("could not cast inputted lidar name to string")
		}

		reading, ok := r.requestParams[reading].(s.TimedLidarReadingResponse)
		if !ok {
			return nil, errors.New("could not cast inputted byte to type sensors.TimedLidarSensorReadingResponse")
		}

		return nil, cf.carto.addLidarReading(lidar, reading)
	case addIMUReading:
		imu, ok := r.requestParams[sensor].(string)
		if !ok {
			return nil, errors.New("could not cast inputted IMU name to string")
		}

		reading, ok := r.requestParams[reading].(s.TimedIMUReadingResponse)
		if !ok {
			return nil, errors.New("could not cast inputted reading to type sensors.TimedIMUSensorReadingResponse")
		}

		return nil, cf.carto.addIMUReading(imu, reading)
	case position:
		return cf.carto.position()
	case internalState:
		return cf.carto.internalState()
	case pointCloudMap:
		return cf.carto.pointCloudMap()
	case runFinalOptimization:
		return nil, cf.carto.runFinalOptimization()
	}
	return nil, fmt.Errorf("no worktype found for: %v", r.requestType)
}

// request wraps calls into C. This function requires the caller to know which RequestTypes
// requires casting to which response values.
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
			msg := "timeout reading from cartographer"
			return nil, multierr.Combine(errors.New(msg), ctx.Err())
		}
	case <-ctx.Done():
		msg := "timeout writing to cartographer"
		return nil, multierr.Combine(errors.New(msg), ctx.Err())
	}
}

// startCGoroutine starts the background goroutine that is responsible for ensuring only one call
// into C is being made at a time.
func (cf *CartoFacade) startCGoroutine(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup) {
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
