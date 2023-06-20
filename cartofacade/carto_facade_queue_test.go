package cartofacade

import (
	"testing"

	"go.viam.com/test"
)

func TestCartoFacadeQueue(t *testing.T) {
	workItem := WorkItem{
		Result:   make(chan interface{}),
		workType: testType,
		inputs:   map[InputType]interface{}{testInput: 1},
	}

	t.Run("Test doWork with test work type", func(t *testing.T) {
		result, err := workItem.DoWork(&Queue{})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, result, test.ShouldResemble, 1)
	})
}
