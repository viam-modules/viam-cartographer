// Package main is a module with a cartographer SLAM service model.
package main

import (
	"context"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/utils"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
)

// Versioning variables which are replaced by LD flags.
var (
	Version     = "development"
	GitRevision = ""
)

func main() {
	utils.ContextualMain(mainWithArgs, golog.NewLogger("cartographerModule"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	// Instantiate the module
	println(Version)
	println(GitRevision)
	return nil
	cartoModule, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}

	// Add the cartographer model to the module
	if err = cartoModule.AddModelFromRegistry(ctx, slam.API, viamcartographer.Model); err != nil {
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
