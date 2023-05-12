package main

/*
	#include "../../c_cpp/slam/slam_service_wrapper.c"
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
	viamCarto := C.viam_carto_New(C.CString("mysensor"))
	fmt.Println("pre init viamCarto.initialized_flag", viamCarto.initialized_flag)

	// == viam_carto_Init ==
	cErrMsg := C.CString("")
	// NOTE: This is probably wrong
	defer C.free(unsafe.Pointer(cErrMsg))
	cErr := C.viam_carto_Init(&viamCarto, &cErrMsg)
	if cErr != 0 {
		return errors.New(C.GoString(cErrMsg))
	}
	fmt.Println("post init viamCarto.initialized_flag", viamCarto.initialized_flag)

	// == viam_carto_GetPosition ==
	response := C.struct_viam_carto_get_position_response{}
	responsePtr := &response
	fmt.Printf("response before %#v\n", response)
	// TODO: How can I be sure position is properly freed? Should the viam_carto
	// library provide a distructor for GetPosition as well?
	cErr = C.viam_carto_GetPosition(&viamCarto, responsePtr, &cErrMsg)
	if cErr != 0 {
		return errors.New(C.GoString(cErrMsg))
	}
	fmt.Printf("response after %#v\n", response)
	fmt.Printf("component_reference: %s\n", C.GoString(response.component_reference))

	// == viam_carto_NewSensorReading ==
	reading := []byte{1, 2, 3, 4, 5}
	cSensorReading := C.viam_carto_NewSensorReading(C.CString("mysensor"), (*C.char)(C.CBytes(reading)), (C.int)(len(reading)))

	// == viam_carto_WriteSensor ==
	fmt.Println("calling viam_carto_WriteSensor ", time.Now())
	cErr = C.viam_carto_WriteSensor(&viamCarto, &cSensorReading, &cErrMsg)
	if cErr != 0 {
		return errors.New(C.GoString(cErrMsg))
	}
	fmt.Println("called viam_carto_WriteSensor", time.Now())

	// == getFred ==
	fmt.Println("calling viam_carto_WriteSensor ", time.Now())

	cErr = C.viam_carto_WriteSensor(&viamCarto, &cSensorReading, &cErrMsg)
	if cErr != 0 {
		return errors.New(C.GoString(cErrMsg))
	}
	fmt.Println("called viam_carto_WriteSensor", time.Now())

	return nil
}
