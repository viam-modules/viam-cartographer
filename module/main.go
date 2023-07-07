// Package main is a module with a cartographer SLAM service model.
package main

import (
	"context"
	"strings"

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
	utils.ContextualMain(mainWithArgs, module.NewLoggerFromArgs("cartographerModule"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	var versionFields []interface{}
	if Version != "" {
		versionFields = append(versionFields, "version", Version)
	}
	if GitRevision != "" {
		versionFields = append(versionFields, "git_rev", GitRevision)
	}
	if len(versionFields) != 0 {
		logger.Infow(viamcartographer.Model.String(), versionFields...)
	} else {
		logger.Info(viamcartographer.Model.String() + " built from source; version unknown")
	}

	if len(args) == 2 && strings.HasSuffix(args[1], "-version") {
		return nil
	}

	if err := viamcartographer.InitCartoLib(logger); err != nil {
		return err
	}
	defer utils.UncheckedErrorFunc(viamcartographer.TerminateCartoLib)

	// Instantiate the module
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
