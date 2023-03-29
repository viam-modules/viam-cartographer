// Package main is a module with a cartographer SLAM service model.
package main

import (
	"context"

	"github.com/edaniels/golog"
	viamcartographer "github.com/viamrobotics/viam-cartographer"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/utils"
)

func main() {
	utils.ContextualMain(mainWithArgs, golog.NewLogger("cartographerModule"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	// Instantiate the module
	cartoModule, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}

	// Add the cartographer model to the module
	if err = cartoModule.AddModelFromRegistry(ctx, slam.Subtype, viamcartographer.Model); err != nil {
		return err
	}

	// Start the module
	err = cartoModule.Start(ctx)
	defer cartoModule.Close(ctx)
	if err != nil {
		return err
	}
	<-ctx.Done()
	return nil
}
