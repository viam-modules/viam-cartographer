// Package cartofacadequeue defines all necessary components to enforce one call into c per cartofacade.
package cartofacadequeue

import (
	"context"
	"fmt"
	"sync"
	"time"

	cartofacade "github.com/viamrobotics/viam-cartographer/viam-cartographer/src/carto_facade_go"
	goutils "go.viam.com/utils"
)

// WorkType defines the grpc call that is being made.
type WorkType int64

const (
	// Initialize can be used to represent the viam_carto_init call in c.
	Initialize WorkType = iota
	// GetPosition can be used to represent the viam_carto_get_position call in c.
	GetPosition
	// GetInternalState can be used to represent the viam_carto_get_internal_state call in c.
	GetInternalState
	// GetPointCloudMap can be used to represent the viam_carto_get_point_cloud_map in c.
	GetPointCloudMap
	// Terminate can be used to represent the viam_carto_terminate in c.
	Terminate

	// TestType can be used to represent a test.
	TestType
)

// InputType defines the type being provided as input to the work.
type InputType int64

const (
	// Name represents the name input into c funcs.
	Name InputType = iota
	// Date represents the date input into c funcs.
	Date

	// TestInput represents a test input.
	TestInput
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
	cViamCartoLib cartofacade.CViamCartoLib,
	cViamCarto cartofacade.CViamCarto,
) (interface{}, error) {
	// TODO: logic for all grpc calls
	switch w.workType {
	case Initialize:
		return cartofacade.NewViamCarto(cViamCartoLib)
	case GetPosition:
		return nil, fmt.Errorf("no worktype found for: %v", w.workType)
	case GetInternalState:
		return nil, fmt.Errorf("no worktype found for: %v", w.workType)
	case GetPointCloudMap:
		return nil, fmt.Errorf("no worktype found for: %v", w.workType)
	case Terminate:
		return cViamCarto.TerminateViamCarto(ctx), nil
	case TestType:
		return w.inputs[TestInput], nil
	}
	return nil, fmt.Errorf("no worktype found for: %v", w.workType)
}

// CartoFacadeQueue represents a queue to consume work from and enforce one call into C at a time.
type CartoFacadeQueue struct {
	Queue         chan WorkItem
	CViamCartoLib *cartofacade.CViamCartoLib
	CViamCarto    cartofacade.CViamCarto
}

// NewCartoFacadeQueue instantiates the CartoFacadeQueue struct.
func NewCartoFacadeQueue() CartoFacadeQueue {
	return CartoFacadeQueue{Queue: make(chan WorkItem)}
}

// HandleIncomingRequest puts incoming requests on the queue and consumes from queue.
func (cfq *CartoFacadeQueue) HandleIncomingRequest(ctx context.Context, workType WorkType, inputs map[InputType]interface{}) interface{} {
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
	case cfq.Queue <- work:
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

func (cfq *CartoFacadeQueue) StartBackgroundWorker(ctx context.Context, activeBackgroundWorkers *sync.WaitGroup) {
	activeBackgroundWorkers.Add(1)

	goutils.PanicCapturingGo(func() {
		defer activeBackgroundWorkers.Done()
		for {
			select {
			case workToDo := <-cfq.Queue:
				result, err := workToDo.DoWork(ctx, *cfq.CViamCartoLib, cfq.CViamCarto)
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
