package main

/*
	#include "../viam-cartographer/src/slam_service/slam_service_wrapper.c"
	#include <stdlib.h>
*/
import "C"
import (
	"context"
	"errors"
	"fmt"
	"unsafe"

	"github.com/edaniels/golog"
	"go.viam.com/utils"
)

// Arguments for the command.
type Arguments struct {
	TripValgrind         bool `flag:"trip_valgrind,usage=trip valgrind"`
	TripAddressSanitizer bool `flag:"trip_address_sanitizer,usage=trip address sanitizer"`
	API                  bool `flag:"api,usage=exercise api"`
}

func main() {
	utils.ContextualMain(mainWithArgs, golog.NewLogger("cartographerModule"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	argsParsed, err := parse(args, logger)
	if err != nil {
		return err
	}

	if argsParsed.TripValgrind {
		fmt.Println("Tripping Valgrind")
		return tripValgrind(logger)
	}

	if argsParsed.TripAddressSanitizer {
		fmt.Println("Tripping AddressSanitizer")
		return tripAddressSanitizer(logger)
	}

	if argsParsed.API {
		fmt.Println("Exercising API")
		return api(logger)
	}

	return cleanRun(logger)
}

func api(logger golog.Logger) error {
	viamCartoPtr := &C.struct_viam_carto{sensors: C.CString("mysensor")}
	cErrMsg := C.CString("")
	// NOTE: This is probably wrong
	defer C.free(unsafe.Pointer(cErrMsg))
	cErr := C.viam_carto_Init(&viamCartoPtr, &cErrMsg)
	goErr := C.GoString(cErrMsg)
	if cErr != 0 {
		return errors.New(goErr)
	}
	// response := C.struct_viam_carto_get_position_response{}
	// cErr = C.viam_carto_GetPosition(&viamCartoPtr, response, &cErrMsg)

	return nil
}

func tripValgrind(logger golog.Logger) error {
	C.trip_valgrind()

	return nil
}

func tripAddressSanitizer(logger golog.Logger) error {
	C.trip_address_sanitizer()

	return nil
}

func cleanRun(logger golog.Logger) error {
	fmt.Println("Clean Run")
	return nil
}

func test() {
	a, b := 1, 2
	// Test cgo
	for i := 1; i < 5; i++ {
		if _, err := C.Hello(); err != nil {
			fmt.Println("error calling Hello function: " + err.Error())
		}

		aC := C.int(a)
		bC := C.int(b)
		mySum, err := C.sum(aC, bC)
		if err != nil {
			fmt.Println("error calling sum function: " + err.Error())
		}
		res := int(mySum)
		fmt.Printf("Sum of %d + %d is %d\n", a, b, res)
		a++
		b++

		myString := "hello there, this is Kat"
		myStringC := C.CString(myString)
		defer C.free(unsafe.Pointer(myStringC))
		startIndex, endIndex := 0, len(myString)-1
		startIndexC := C.int(startIndex)
		endIndexC := C.int(endIndex)

		if _, err := C.reverse(myStringC, startIndexC, endIndexC); err != nil {
			fmt.Println("error calling reverse function: " + err.Error())
		}
		myStringReversed := C.GoString(myStringC)
		fmt.Println("strings reversed using C: ", myString, " --> ", myStringReversed)
	}
}

func parse(args []string, logger golog.Logger) (Arguments, error) {
	var argsParsed Arguments
	if err := utils.ParseFlags(args, &argsParsed); err != nil {
		return argsParsed, err
	}
	return argsParsed, nil
}
