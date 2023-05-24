package main

/*
  #cgo LDFLAGS: -v -L../viam-cartographer/build  -lviam-cartographer -lstdc++ -lpthread
	#include "../viam-cartographer/src/c-api/slam_service_c.h"
*/
import "C"
import "fmt"

func main() {
	fmt.Println(C.TestCgoC(1))
	fmt.Println(C.TestCgoC(5))

	fmt.Println(C.TestCgoC(100))
	fmt.Println(C.TestCgoC(101))
	fmt.Println(C.TestCgoC(199))
	fmt.Println(C.TestCgoC(998))
}
