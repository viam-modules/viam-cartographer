package cartofacadequeue

import (
	"context"
	"testing"

	"go.viam.com/test"

	cartoFacade "github.com/viamrobotics/viam-cartographer/viam-cartographer/src/carto_facade_go"
)

func TestViamCartoCGoAPI(t *testing.T) {
	workItem := WorkItem{
		Result:   make(chan interface{}),
		workType: TestType,
		inputs:   map[InputType]interface{}{TestInput: 1},
	}

	// TODO: fill with reasonable log level and verbosity
	cViamCartoLib, err := cartoFacade.NewViamCartoLib(1, 1)
	test.That(t, err, test.ShouldBeNil)

	cViamCarto, err := cartoFacade.NewViamCarto(cViamCartoLib)
	test.That(t, err, test.ShouldBeNil)

	t.Run("Test doWork with test work type", func(t *testing.T) {
		result, err := workItem.DoWork(context.Background(), cViamCartoLib, cViamCarto)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, result, test.ShouldResemble, 1)
	})
}
