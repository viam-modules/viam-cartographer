package cartoFacadeQueue

import (
	"context"
	"fmt"
	"time"

	cartoFacade "github.com/viamrobotics/viam-cartographer/viam-cartographer/src/carto_facade_go"
)

type WorkType int64

const (
	Initialize WorkType = iota
	GetPosition
	GetInternalState
	GetPointCloudMap
	Terminate

	TestType
)

type InputType int64

const (
	Name InputType = iota
	Date
	TestInput
)

type WorkItem struct {
	Result   chan interface{}
	workType WorkType
	inputs   map[InputType]interface{}
}

func (w *WorkItem) DoWork(cViamCartoLib cartoFacade.CViamCartoLib, cViamCarto cartoFacade.CViamCarto) (interface{}, error) {
	// TODO: logic for all grpc calls
	if w.workType == Initialize {
		return cartoFacade.NewViamCarto(cViamCartoLib)
	} else if w.workType == TestType {
		fmt.Printf("Do Work Returning: %d \n", w.inputs[TestInput])
		return w.inputs[TestInput], nil
	}
	return nil, fmt.Errorf("No worktype found for: %v", w.workType)
}

type CartoFacadeQueue struct {
	Queue chan WorkItem
}

func NewCartoFacadeQueue() CartoFacadeQueue {
	return CartoFacadeQueue{Queue: make(chan WorkItem)}
}

func (cfq *CartoFacadeQueue) HandleIncomingRequest(ctx context.Context, workType WorkType, inputs map[InputType]interface{}) interface{} {
	//TODO: determine good time for the timeout
	ctx, cancel := context.WithTimeout(ctx, 5*time.Second)
	defer cancel()

	work := WorkItem{
		Result:   make(chan interface{}),
		workType: workType,
		inputs:   inputs,
	}

	// wait until work can get put put on the queue (and timeout if needed)
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
