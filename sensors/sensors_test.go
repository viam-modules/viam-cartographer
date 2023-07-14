// Package s_test implements tests for sensors
package sensors_test

import (
	"context"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.viam.com/test"

	s "github.com/viamrobotics/viam-cartographer/sensors"
	dim2d "github.com/viamrobotics/viam-cartographer/sensors/lidar/dim-2d"
	"github.com/viamrobotics/viam-cartographer/testhelper"
)

func TestValidateGetData(t *testing.T) {
	logger := golog.NewTestLogger(t)
	ctx := context.Background()

	sensors := []string{"good_lidar"}
	goodLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
	test.That(t, err, test.ShouldBeNil)

	sensors = []string{"invalid_sensor"}
	invalidLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
	test.That(t, err, test.ShouldBeNil)

	sensorValidationMaxTimeout := time.Duration(50) * time.Millisecond
	sensorValidationInterval := time.Duration(10) * time.Millisecond

	t.Run("returns nil if a lidar reading succeeds immediately", func(t *testing.T) {
		err := s.ValidateGetData(ctx, goodLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns nil if a lidar reading succeeds within the timeout", func(t *testing.T) {
		sensors = []string{"warming_up_lidar"}
		warmingUpLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
		test.That(t, err, test.ShouldBeNil)

		err = s.ValidateGetData(ctx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns error if no lidar reading succeeds within the timeout", func(t *testing.T) {
		err := s.ValidateGetData(ctx, invalidLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeError, errors.New("ValidateGetData timeout: NextPointCloud error: invalid sensor"))
	})

	t.Run("returns error if no lidar reading succeeds by the time the context is cancelled", func(t *testing.T) {
		cancelledCtx, cancelFunc := context.WithCancel(context.Background())
		cancelFunc()

		sensors = []string{"warming_up_lidar"}
		warmingUpLidar, err := dim2d.NewLidar(ctx, testhelper.SetupDeps(sensors), sensors, logger)
		test.That(t, err, test.ShouldBeNil)

		err = s.ValidateGetData(cancelledCtx, warmingUpLidar, sensorValidationMaxTimeout, sensorValidationInterval, logger)
		test.That(t, err, test.ShouldBeError, context.Canceled)
	})
}
