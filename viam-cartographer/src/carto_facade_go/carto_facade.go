package carto_facade

/*
	#cgo CFLAGS: -I../carto_facade
  	#cgo LDFLAGS: -v -L../../build  -lviam-cartographer -lglog -lstdc++ -lpthread

	#include "../carto_facade/carto_facade.h"
	#include "bstrlib.h"

	viam_carto_lib* alloc_viam_carto_lib() { return (viam_carto_lib*) malloc(sizeof(viam_carto_lib)); }
	void free_alloc_viam_carto_lib(viam_carto_lib* p) { free(p); }

	viam_carto* alloc_viam_carto() { return (viam_carto*) malloc(sizeof(viam_carto)); }
	void free_alloc_viam_carto(viam_carto* p) { free(p); }

	bstring* alloc_bstring_array(size_t len) { return (bstring*) malloc(len * sizeof(bstring)); }
	void free_bstring_array(bstring* p) { free(p); }
*/
import "C"
import (
	"context"
	"errors"
	"fmt"
	"unsafe"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func successfulCCall(status C.int) bool {
	return int(status) == int(C.VIAM_CARTO_SUCCESS)
}

type CViamCartoLib struct {
	viam_carto_lib *C.viam_carto_lib
}

func NewCViamCartoLibObj(miniloglevel int, verbose int) (*CViamCartoLib, error) {
	pVcl := C.alloc_viam_carto_lib()
	defer C.free_alloc_viam_carto_lib(pVcl)

	ppVcl := (**C.viam_carto_lib)(unsafe.Pointer(pVcl))

	status := C.viam_carto_lib_init(ppVcl, C.int(miniloglevel), C.int(verbose))
	if !successfulCCall(status) {
		// TODO: how to make this error useful?
		return nil, errors.New(fmt.Sprintf("Error initializing viam_carto_lib status = %d", status))
	}

	vcl := CViamCartoLib{viam_carto_lib: (*C.viam_carto_lib)(*ppVcl)}

	return &vcl, nil
}

func (vcl *CViamCartoLib) TerminateCViamCartoLibObj() error {
	status := C.viam_carto_lib_terminate((**C.viam_carto_lib)(unsafe.Pointer(vcl)))
	if !successfulCCall(status) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}
	return nil
}

type CViamCarto struct {
	impl *C.viam_carto
}

func get_config() C.viam_carto_config {
	vcc := C.viam_carto_config{}

	sz := 1
	pSensor := C.alloc_bstring_array(C.size_t(sz))
	sensor_slice := unsafe.Slice(pSensor, sz)
	sensor_slice[0] = C.bfromcstr(C.CString("rplidar"))

	vcc.sensors = pSensor
	vcc.sensors_len = C.int(sz)

	return vcc
}

func clean_config(vcc C.viam_carto_config) {
	C.free_bstring_array(vcc.sensors)
}

func NewCViamCartoObj(vcl CViamCartoLib) (*CViamCarto, error) {
	pVc := C.alloc_viam_carto()
	defer C.free_alloc_viam_carto(pVc)

	ppVc := (**C.viam_carto)(unsafe.Pointer(pVc))

	// TODO: Init with data
	vcc := get_config()
	defer clean_config(vcc)
	vcac := C.viam_carto_algo_config{}

	status := C.viam_carto_init(ppVc, vcl.viam_carto_lib, vcc, vcac)
	if !successfulCCall(status) {
		// TODO: how to make this error useful?
		return nil, errors.New(fmt.Sprintf("Error initializing viam_carto status = %d", status))
	}

	vc := CViamCarto{impl: (*C.viam_carto)(*ppVc)}
	return &vc, nil
}

func (vc *CViamCarto) Start(ctx context.Context) error {
	status := C.viam_carto_start(vc.impl)

	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	return nil
}

func (vc *CViamCarto) Stop(ctx context.Context) error {
	status := C.viam_carto_stop(vc.impl)

	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	return nil
}

func (vc *CViamCarto) TerminateViamCarto(ctx context.Context) error {
	status := C.viam_carto_terminate(&vc.impl)

	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	return nil
}

type CViamCartoAddSensorReading struct {
	vcasr C.viam_carto_sensor_reading
}

func (vc *CViamCarto) AddSensorReading(ctx context.Context) error {
	resp := CViamCartoAddSensorReading{vcasr: C.viam_carto_sensor_reading{}}

	status := C.viam_carto_add_sensor_reading(vc.impl, &resp.vcasr)

	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	status = C.viam_carto_add_sensor_reading_destroy(&resp.vcasr)
	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	return nil
}

type CViamCartoGetPositionResponse struct {
	vcgpr C.viam_carto_get_position_response
}

func (vc *CViamCarto) GetPosition(ctx context.Context) (spatialmath.Pose, string, error) {
	resp := CViamCartoGetPositionResponse{vcgpr: C.viam_carto_get_position_response{}}

	status := C.viam_carto_get_position(vc.impl, &resp.vcgpr)

	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return nil, "", errors.New(string(status))
	}

	pos := r3.Vector{
		X: float64(resp.vcgpr.x),
		Y: float64(resp.vcgpr.y),
		Z: float64(resp.vcgpr.z)}

	// TODO: What should be done with quat info?
	// quat := &spatialmath.Quaternion{
	// 	Real: float64(resp.vcgpr.real),
	// 	Imag: float64(resp.vcgpr.imag),
	// 	Jmag: float64(resp.vcgpr.jmag),
	// 	Kmag: float64(resp.vcgpr.kmag)}

	ori := &spatialmath.OrientationVectorDegrees{
		OX: float64(resp.vcgpr.o_x),
		OY: float64(resp.vcgpr.o_y),
		OZ: float64(resp.vcgpr.o_z)}

	status = C.viam_carto_get_position_response_destroy(&resp.vcgpr)
	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return nil, "", errors.New(string(status))
	}

	// TODO: fill component reference in?
	return spatialmath.NewPose(pos, ori), "", nil
}

type CViamCartoGetPointCloudMapResponse struct {
	vcgpcmr C.viam_carto_get_point_cloud_map_response
}

func (vc *CViamCarto) GetPointCloudMap(ctx context.Context) (func() ([]byte, error), error) {
	resp := CViamCartoGetPointCloudMapResponse{vcgpcmr: C.viam_carto_get_point_cloud_map_response{}}

	status := C.viam_carto_get_point_cloud_map(vc.impl, &resp.vcgpcmr)

	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return nil, errors.New(string(status))
	}

	status = C.viam_carto_get_point_cloud_map_response_destroy(&resp.vcgpcmr)
	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return nil, errors.New(string(status))
	}

	return nil, nil
}

type CViamCartoGetInternalStateResponse struct {
	vcgisr C.viam_carto_get_internal_state_response
}

func (vc *CViamCarto) GetInternalState(ctx context.Context) (func() ([]byte, error), error) {
	resp := CViamCartoGetInternalStateResponse{vcgisr: C.viam_carto_get_internal_state_response{}}

	status := C.viam_carto_get_internal_state(vc.impl, &resp.vcgisr)

	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return nil, errors.New(string(status))
	}

	status = C.viam_carto_get_internal_state_response_destroy(&resp.vcgisr)
	if int(status) != int(C.VIAM_CARTO_SUCCESS) {
		// TODO: how to make this error useful?
		return nil, errors.New(string(status))
	}

	return nil, nil
}
