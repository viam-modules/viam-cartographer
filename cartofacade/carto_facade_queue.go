package cartofacade

import (
	"context"
	"fmt"
	"sync"
	"time"

	goutils "go.viam.com/utils"
)

// WorkType defines the grpc call that is being made.
type WorkType int64

const (
	// Initialize can be used to represent the viam_carto_init call in c.
	Initialize WorkType = iota
	// Position can be used to represent the viam_carto_get_position call in c.
	Position
	// InternalState can be used to represent the viam_carto_get_internal_state call in c.
	InternalState
	// PointCloudMap can be used to represent the viam_carto_get_point_cloud_map in c.
	PointCloudMap
	// Terminate can be used to represent the viam_carto_terminate in c.
	Terminate

	// testType can be used to represent a test.
	testType
)

// InputType defines the type being provided as input to the work.
type InputType int64

const (
	// Name represents the name input into c funcs.
	Name InputType = iota
	// Date represents the date input into c funcs.
	Date

	// testInput represents a test input.
	testInput
)

// WorkItem defines one piece of work that can be put on the queue.
type WorkItem struct {
	Result   chan interface{}
	workType WorkType
	inputs   map[InputType]interface{}
}

// DoWork provides the logic to call the correct cgo functions with the correct input.
func (w *WorkItem) DoWork(
	ctx context.Context,
	q *Queue,
) (interface{}, error) {
	// TODO: logic for all grpc calls
	switch w.workType {
	case Initialize:
		return New(q.CartoConfig, q.CartoAlgoConfig, *q.CartoLib)
	case Position:
		return q.Carto.GetPosition()
	case InternalState:
		return q.Carto.GetInternalState()
	case PointCloudMap:
		return q.Carto.GetPointCloudMap()
	case Terminate:
		return q.Carto.Terminate(), nil
	case testType:
		return w.inputs[testInput], nil
	}
	return nil, fmt.Errorf("no worktype found for: %v", w.workType)
}

// Queue represents a queue to consume work from and enforce one call into C at a time.
type Queue struct {
	WorkChannel     chan WorkItem
	CartoLib        *CartoLib
	Carto           Carto
	CartoConfig     CartoConfig
	CartoAlgoConfig CartoAlgoConfig
}

// NewQueue instantiates the Queue struct.
func NewQueue(cartoLib *CartoLib, cartoCfg CartoConfig, cartoAlgoCfg CartoAlgoConfig) Queue {
	return Queue{
		WorkChannel:     make(chan WorkItem),
		Carto:           Carto{},
		CartoLib:        cartoLib,
		CartoConfig:     cartoCfg,
		CartoAlgoConfig: cartoAlgoCfg,
	}
}

// HandleIncomingRequest puts incoming requests on the queue and consumes from queue.
func (q *Queue) HandleIncomingRequest(ctx context.Context, workType WorkType, inputs map[InputType]interface{}) interface{} {
	// TODO: determine good time for the timeout
	ctx, cancel := context.WithTimeout(ctx, 5*time.Second)
	defer cancel()

	work := WorkItem{
		Result:   make(chan interface{}),
		workType: workType,
		inputs:   inputs,
	}

	// wait until work can get put on the queue (and timeout if needed)
	select {
	case q.WorkChannel <- work:
		// wait until result is put on result queue (and timeout if needed)
		select {
		case result := <-work.Result:
			return result
		case <-ctx.Done():
			return nil
		}
	case <-ctx.Done():
		return nil
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
			case workToDo := <-q.WorkChannel:
				result, err := workToDo.DoWork(ctx, q)
				if err == nil {
					workToDo.Result <- result
				} else {
					workToDo.Result <- err
				}
			case <-ctx.Done():
				return
			}
		}
	})
}
