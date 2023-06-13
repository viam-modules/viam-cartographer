// Package main is a module with a cartographer SLAM service model.
package main

import (
	"context"
	"fmt"
	"strings"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/utils"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	cartofacade "github.com/viamrobotics/viam-cartographer/viam-cartographer/src/carto_facade_go"
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

	// Instantiate the module
	cartoModule, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}

	// Add the cartographer model to the module
	if err = cartoModule.AddModelFromRegistry(ctx, slam.API, viamcartographer.Model); err != nil {
		return err
	}

	// TODO: instantiate viam_carto_lib with correct loglevels and verbosity
	vcl, err := cartofacade.NewViamCartoLib(1, 1)
	if err == nil {
		return fmt.Errorf("unable to initialize viam_carto_lib.")
	}
	viamcartographer.ViamCartoLib = &vcl

	// Start the module
	err = cartoModule.Start(ctx)
	defer cartoModule.Close(ctx)
	if err != nil {
		return err
	}

	defer func() {
		err := vcl.TerminateViamCartoLib()
		if err != nil {
			logger.Error(err)
		}
	}()

	<-ctx.Done()
	return nil
}
