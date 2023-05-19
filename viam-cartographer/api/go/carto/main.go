package main

/*
	#include "../../src/slam/viam_carto.c"
*/
import "C"
import (
	"context"
	"errors"
	"fmt"
	"github.com/edaniels/golog"
	"go.viam.com/utils"
	"time"
	"unsafe"
)

func main() {
	utils.ContextualMain(mainWithArgs, golog.NewLogger("carto-go"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {

	return nil
}
