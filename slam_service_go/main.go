package main

/*
  #cgo LDFLAGS: -v -L../viam-cartographer/build  -lviam-cartographer  -lstdc++ -lpthread
	#include "../viam-cartographer/src/carto_facade/carto_facade.h"
*/
import "C"
import "fmt"

type CViamCarto struct {
	viam_carto *C.viam_carto
}

type CViamCartoGetPositionResponse struct {
	vcgpr C.viam_carto_get_position_response
}

func main() {
	vc := CViamCarto{viam_carto: &C.viam_carto{}}
	fmt.Printf("%#v\n", vc)
	vcgpr := CViamCartoGetPositionResponse{vcgpr: C.viam_carto_get_position_response{}}
	fmt.Printf("%#v\n", vcgpr)

	fmt.Println(C.viam_carto_get_position(vc.viam_carto, &vcgpr.vcgpr))
}
