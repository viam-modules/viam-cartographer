package main

/*
  #cgo LDFLAGS: -v -L../../build -lcounter -lstdc++ -lpthread
	#include "../../src/counter/mycounter.h"
	#include <stdlib.h>
*/
import "C"
import (
	"fmt"
)

func main() {
	c := C.c_counter()
	fmt.Println("From Go:")
	for i := 0; i < 5; i++ {
		fmt.Printf("%d counter incremented %d\n", i, C.c_inc(c, 1))
	}
}
