// Package main is a module with a cartographer SLAM service model.
package main

import (
	"context"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/utils"
	goutils "go.viam.com/utils"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
)

// Versioning variables which are replaced by LD flags.
var (
	Version     = "development"
	GitRevision = ""
)

// Arguments for the command.
type Arguments struct {
	Version bool `flag:"version,usage=print version"`
}

func main() {
	utils.ContextualMain(mainWithArgs, golog.NewLogger("cartographerModule"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	argsParsed := printVersion(args, logger)
	if argsParsed.Version {
		return nil
	}

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

func printVersion(args []string, logger golog.Logger) Arguments {
	var argsParsed Arguments
	// Don't propagate error if there are additional flags
	// that are not recognized. Otherwise the module fails
	// under normal operation.
	goutils.UncheckedError(utils.ParseFlags(args, &argsParsed))

	// Always log the version, return early if the '-version' flag was provided
	// fmt.Println would be better but fails linting. Good enough.
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

	return argsParsed
}
