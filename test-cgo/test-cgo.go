package main

/*
	#include "../viam-cartographer/src/slam_service/slam_service_wrapper.c"
	#include <stdlib.h>
*/
import "C"
import (
	"fmt"
	"unsafe"
)

func main() {
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
