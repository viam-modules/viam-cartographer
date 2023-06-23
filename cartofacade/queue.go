package cartofacade

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"time"

	cgoApi "github.com/viamrobotics/viam-cartographer/cartofacade/internal/capi"
	goutils "go.viam.com/utils"
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
	// Reading represents a lidar reading input into c funcs.
	Reading InputType = iota
	// Timestamp represents the timestamp input into c funcs.
	Timestamp
)

// ResultType defines the type being provided as input to the work.
type ResultType int64

const (
	// ByteSlice represents the go type []Byte.
	ByteSlice ResultType = iota
	// PositionResponse represents the custom struct defined for getPosition responses.
	PositionResponse
	// CartoType represents the custom struct defined for Carto structs.
	CartoType
	// Nil represents the go type nil.
	Nil
	// Error represents the go type error.
	Error
)

// Result defines the result of one piece of work that can be put on the result channel.
type Result struct {
	result     interface{}
	resultType ResultType
}

// Queue represents a queue to consume work from and enforce one call into C at a time.
type Queue struct {
	WorkChannel     chan WorkItem
	CartoLib        cgoApi.CartoLibInterface
	Carto           cgoApi.CartoInterface
	CartoConfig     cgoApi.CartoConfig
	CartoAlgoConfig cgoApi.CartoAlgoConfig
}

// WorkItemInterface defines one piece of work that can be put on the queue.
type WorkItemInterface interface {
	DoWork(q *Queue) (interface{}, ResultType, error)
}

// WorkItem defines one piece of work that can be put on the queue.
type WorkItem struct {
	Result   chan Result
	workType WorkType
	inputs   map[InputType]interface{}
}

// NewQueue instantiates the Queue struct.
func NewQueue(cartoLib cgoApi.CartoLibInterface, cartoCfg cgoApi.CartoConfig, cartoAlgoCfg cgoApi.CartoAlgoConfig) Queue {
	return Queue{
		WorkChannel:     make(chan WorkItem),
		Carto:           &cgoApi.Carto{},
		CartoLib:        cartoLib,
		CartoConfig:     cartoCfg,
		CartoAlgoConfig: cartoAlgoCfg,
	}

}

// DoWork provides the logic to call the correct cgo functions with the correct input.
func (w *WorkItem) DoWork(
	q *Queue,
) (interface{}, ResultType, error) {
	switch w.workType {
	case Initialize:
		carto, err := cgoApi.New(q.CartoConfig, q.CartoAlgoConfig, q.CartoLib)
		return carto, CartoType, err
	case Start:
		return nil, Nil, q.Carto.Start()
	case Stop:
		return nil, Nil, q.Carto.Stop()
	case Terminate:
		return nil, Nil, q.Carto.Terminate()
	case AddSensorReading:
		reading, ok := w.inputs[Reading].([]byte)
		if !ok {
			return nil, Nil, errors.New("could not cast inputted byte to byte slice")
		}

		timestamp, ok := w.inputs[Timestamp].(time.Time)
		if !ok {
			return nil, Nil, errors.New("could not cast inputted timestamp to times.Time")
		}

		return nil, Nil, q.Carto.AddSensorReading(reading, timestamp)
	case Position:
		positionResponse, err := q.Carto.GetPosition()
		return positionResponse, PositionResponse, err
	case InternalState:
		internalState, err := q.Carto.GetInternalState()
		return internalState, ByteSlice, err
	case PointCloudMap:
		pointCloudMap, err := q.Carto.GetPointCloudMap()
		return pointCloudMap, ByteSlice, err
	}
	return nil, Nil, fmt.Errorf("no worktype found for: %v", w.workType)
}

// Request puts incoming requests on the queue and consumes from queue.
func (q *Queue) Request(ctxParent context.Context, workType WorkType, inputs map[InputType]interface{}, timeout time.Duration) (Result, error) {
	ctx, cancel := context.WithTimeout(ctxParent, timeout)
	defer cancel()

	work := WorkItem{
		Result:   make(chan Result),
		workType: workType,
		inputs:   inputs,
	}

	// wait until work can get put on the queue (and timeout if needed)
	select {
	case q.WorkChannel <- work:
		select {
		case result := <-work.Result:
			if result.resultType == Error {
				return Result{}, result.result.(error)
			}
			return result, nil
		case <-ctx.Done():
			fmt.Println("Request: ctx.Done()")
			return Result{}, errors.New("timeout has occurred while trying to read request from cartofacade")
		}
	case <-ctx.Done():
		return Result{}, errors.New("timeout has occurred while trying to write request to cartofacade")
	}
}

// StartBackgroundWorker starts the background goroutine that is responsible for putting work
// onto the queue and consuming from the queue.
func (q *Queue) StartBackgroundWorker(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup) {
	activeBackgroundWorkers.Add(1)
	goutils.PanicCapturingGo(func() {
		defer activeBackgroundWorkers.Done()

		for {
			select {
			case <-ctx.Done():
				return
			case workToDo := <-q.WorkChannel:
				result, resultType, err := workToDo.DoWork(q)
				if err != nil {
					workToDo.Result <- Result{result: err, resultType: Error}
				} else {
					workToDo.Result <- Result{result: result, resultType: resultType}
				}
			}
		}
	})
}
