// Package main is a module with a cartographer SLAM service model.
package main

/*
	#include "../viam-cartographer/src/slam_service/slam_service_wrapper.c"
	#include <stdlib.h>
*/
import "C"
import (
	"context"
	"fmt"
	"unsafe"

	"github.com/edaniels/golog"
	// "go.viam.com/rdk/module"
	// "go.viam.com/rdk/services/slam"
	"go.viam.com/utils"
	// viamcartographer "github.com/viamrobotics/viam-cartographer"
)

func main() {
	utils.ContextualMain(mainWithArgs, golog.NewLogger("cartographerModule"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	fmt.Println("===fmtTestCHello===")
	fmtTestCHello()

	fmt.Println("===loggerTestCHello===")
	loggerTestCHello(logger)

	fmt.Println("===fmtTestCReverse===")
	fmtTestCReverse()

	fmt.Println("===loggerTestCReverse===")
	loggerTestCReverse(logger)
	// Instantiate the module
	// cartoModule, err := module.NewModuleFromArgs(ctx, logger)
	// if err != nil {
	// 	return err
	// }

	// // Add the cartographer model to the module
	// if err = cartoModule.AddModelFromRegistry(ctx, slam.API, viamcartographer.Model); err != nil {
	// 	return err
	// }

	// // Start the module
	// err = cartoModule.Start(ctx)
	// defer cartoModule.Close(ctx)
	// if err != nil {
	// 	return err
	// }
	// <-ctx.Done()
	return nil
}

func fmtTestCHello() {
	fmt.Println("before C.Hello()")
	if _, err := C.Hello(); err != nil {
		fmt.Println("error calling Hello function: " + err.Error())
	}
	fmt.Println("after C.Hello()")
}

func loggerTestCHello(logger golog.Logger) {
	logger.Info("before C.Hello()")
	if _, err := C.Hello(); err != nil {
		logger.Info("error calling Hello function: " + err.Error())
	}
	logger.Info("after C.Hello()")
}

func fmtTestCReverse() {
	fmt.Println("before reverse")
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

func loggerTestCReverse(logger golog.Logger) {
	logger.Info("before reverse")
	myString := "hello there, this is Kat"
	myStringC := C.CString(myString)
	defer C.free(unsafe.Pointer(myStringC))
	startIndex, endIndex := 0, len(myString)-1
	startIndexC := C.int(startIndex)
	endIndexC := C.int(endIndex)

	if _, err := C.reverse(myStringC, startIndexC, endIndexC); err != nil {
		logger.Info("error calling reverse function: " + err.Error())
	}
	myStringReversed := C.GoString(myStringC)
	logger.Info("strings reversed using C: ", myString, " --> ", myStringReversed)
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
