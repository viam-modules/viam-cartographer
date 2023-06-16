package cartofacade

import (
	"context"
	"testing"

	"go.viam.com/test"
)

func TestCartoFacadeQueue(t *testing.T) {
	workItem := WorkItem{
		Result:   make(chan interface{}),
		workType: TestType,
		inputs:   map[InputType]interface{}{TestInput: 1},
	}

	t.Run("Test doWork with test work type", func(t *testing.T) {
		result, err := workItem.DoWork(context.Background(), &CartoFacadeQueue{})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, result, test.ShouldResemble, 1)
	})
}
