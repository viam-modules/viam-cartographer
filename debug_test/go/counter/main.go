package main

/*
  #cgo LDFLAGS: -v -L../../c_cpp/build -lcounter -lstdc++ -lpthread
	#include "../../c_cpp/counter/mycounter.h"
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
