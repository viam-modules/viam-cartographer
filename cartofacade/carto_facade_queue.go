package cartofacade

import (
	"context"
	"errors"
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

	// testType can be used to represent a test.
	testType
)

// InputType defines the type being provided as input to the work.
type InputType int64

const (
	// Reading represents a lidar reading input into c funcs.
	Reading InputType = iota
	// Timestamp represents the timestamp input into c funcs.
	Timestamp

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
	q *Queue,
) (interface{}, error) {
	switch w.workType {
	case Initialize:
		return New(q.CartoConfig, q.CartoAlgoConfig, *q.CartoLib)
	case Start:
		return nil, q.Carto.Start()
	case Stop:
		return nil, q.Carto.Stop()
	case Terminate:
		return q.Carto.Terminate(), nil
	case AddSensorReading:
		reading, ok := w.inputs[Reading].([]byte)
		if !ok {
			return nil, errors.New("could not cast inputted byte to byte slice")
		}

		timestamp, ok := w.inputs[Timestamp].(time.Time)
		if !ok {
			return nil, errors.New("could not cast inputted timestamp to times.Time")
		}

		return nil, q.Carto.AddSensorReading(reading, timestamp)
	case Position:
		return q.Carto.GetPosition()
	case InternalState:
		return q.Carto.GetInternalState()
	case PointCloudMap:
		return q.Carto.GetPointCloudMap()
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
	// Question: What is reasonable timeout period?
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
				result, err := workToDo.DoWork(q)
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
