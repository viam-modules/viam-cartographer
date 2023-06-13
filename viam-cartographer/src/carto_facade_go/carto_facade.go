package cartoFacade

/*
	#cgo CFLAGS: -I../carto_facade
  	#cgo LDFLAGS: -v -L../../build  -lviam-cartographer -lglog -lstdc++ -lpthread

	#include "../carto_facade/carto_facade.h"
	// TODO: confirm bstring import is necessary and why
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

func successfulStatus(status C.int) bool {
	return int(status) == int(C.VIAM_CARTO_SUCCESS)
}

// CViamCartoLib is a struct to hold the c type viam_carto_lib
type CViamCartoLib struct {
	viamCartolib *C.viam_carto_lib
}

// NewViamCartoLib calls viam_carto_lib_init and returns a pointer to a viam carto lib object.
func NewViamCartoLib(miniloglevel, verbose int) (CViamCartoLib, error) {
	pVcl := C.alloc_viam_carto_lib()
	defer C.free_alloc_viam_carto_lib(pVcl)

	ppVcl := (**C.viam_carto_lib)(unsafe.Pointer(pVcl))

	status := C.viam_carto_lib_init(ppVcl, C.int(miniloglevel), C.int(verbose))
	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return CViamCartoLib{}, fmt.Errorf("error initializing viam_carto_lib status = %d", status)
	}

	vcl := CViamCartoLib{viamCartolib: (*ppVcl)}

	return vcl, nil
}

// TerminateViamCartoLib calls viam_carto_lib_terminate to clean up memory for viam carto lib.
func (vcl *CViamCartoLib) TerminateViamCartoLib() error {
	status := C.viam_carto_lib_terminate((**C.viam_carto_lib)(unsafe.Pointer(vcl)))
	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}
	return nil
}

// CViamCarto is a struct to hold the c type viam_carto
type CViamCarto struct {
	viamCarto *C.viam_carto
}

// TODO: properly pass config
func getConfig() C.viam_carto_config {
	vcc := C.viam_carto_config{}

	sz := 1
	pSensor := C.alloc_bstring_array(C.size_t(sz))
	sensorSlice := unsafe.Slice(pSensor, sz)
	sensorSlice[0] = C.bfromcstr(C.CString("rplidar"))

	vcc.sensors = pSensor
	vcc.sensors_len = C.int(sz)

	dataDir := C.bfromcstr(C.CString("data_dir"))
	vcc.data_dir = dataDir

	componentReference := C.bfromcstr(C.CString("comp_ref"))
	vcc.component_reference = componentReference

	return vcc
}

func cleanConfig(vcc C.viam_carto_config) {
	C.free_bstring_array(vcc.sensors)
}

// NewViamCarto calls viam_carto_init and returns a pointer to a viam carto object.
func NewViamCarto(vcl CViamCartoLib) (*CViamCarto, error) {
	// TODO: get vcl as a param & initialize c queue channel here
	pVc := C.alloc_viam_carto()
	defer C.free_alloc_viam_carto(pVc)

	ppVc := (**C.viam_carto)(unsafe.Pointer(pVc))

	// TODO: Init with data
	vcc := getConfig()
	defer cleanConfig(vcc)
	vcac := C.viam_carto_algo_config{}

	status := C.viam_carto_init(ppVc, vcl.viamCartolib, vcc, vcac)
	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return nil, fmt.Errorf("error initializing viam_carto status = %d", status)
	}

	vc := CViamCarto{viamCarto: (*C.viam_carto)(*ppVc)}
	return &vc, nil
}

// Start is a wrapper for viam_carto_start
func (vc *CViamCarto) Start(ctx context.Context) error {
	status := C.viam_carto_start(vc.viamCarto)

	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	return nil
}

// Stop is a wrapper for viam_carto_stop
func (vc *CViamCarto) Stop(ctx context.Context) error {
	status := C.viam_carto_stop(vc.viamCarto)

	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	return nil
}

// TerminateViamCarto calls viam_carto_terminate to clean up memory for viam carto
func (vc *CViamCarto) TerminateViamCarto(ctx context.Context) error {
	status := C.viam_carto_terminate(&vc.viamCarto)

	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	return nil
}

// CViamCartoAddSensorReading is a struct to hold the c type viam_carto_sensor_reading
type CViamCartoAddSensorReading struct {
	vcasr C.viam_carto_sensor_reading
}

// AddSensorReading is a wrapper for viam_carto_add_sensor_reading
func (vc *CViamCarto) AddSensorReading(ctx context.Context) error {
	resp := CViamCartoAddSensorReading{vcasr: C.viam_carto_sensor_reading{}}

	status := C.viam_carto_add_sensor_reading(vc.viamCarto, &resp.vcasr)

	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	status = C.viam_carto_add_sensor_reading_destroy(&resp.vcasr)
	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return errors.New(string(status))
	}

	return nil
}

// CViamCartoGetPositionResponse is a struct to hold the c type viam_carto_get_position_response
type CViamCartoGetPositionResponse struct {
	vcgpr C.viam_carto_get_position_response
}

// GetPosition is a wrapper for viam_carto_get_position
func (vc *CViamCarto) GetPosition(ctx context.Context) (spatialmath.Pose, string, error) {
	resp := CViamCartoGetPositionResponse{vcgpr: C.viam_carto_get_position_response{}}

	status := C.viam_carto_get_position(vc.viamCarto, &resp.vcgpr)

	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return nil, "", errors.New(string(status))
	}

	pos := r3.Vector{
		X: float64(resp.vcgpr.x),
		Y: float64(resp.vcgpr.y),
		Z: float64(resp.vcgpr.z),
	}

	// TODO: What should be done with quat info?
	// quat := &spatialmath.Quaternion{
	// 	Real: float64(resp.vcgpr.real),
	// 	Imag: float64(resp.vcgpr.imag),
	// 	Jmag: float64(resp.vcgpr.jmag),
	// 	Kmag: float64(resp.vcgpr.kmag)}

	ori := &spatialmath.OrientationVectorDegrees{
		OX: float64(resp.vcgpr.o_x),
		OY: float64(resp.vcgpr.o_y),
		OZ: float64(resp.vcgpr.o_z),
	}

	status = C.viam_carto_get_position_response_destroy(&resp.vcgpr)
	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return nil, "", errors.New(string(status))
	}

	// TODO: fill component reference in?
	return spatialmath.NewPose(pos, ori), "", nil
}

// CViamCartoGetPointCloudMapResponse is a struct to hold the c type viam_carto_get_point_cloud_map_response
type CViamCartoGetPointCloudMapResponse struct {
	vcgpcmr C.viam_carto_get_point_cloud_map_response
}

// GetPointCloudMap is a wrapper for viam_carto_get_point_cloud_map
func (vc *CViamCarto) GetPointCloudMap(ctx context.Context) (func() ([]byte, error), error) {
	resp := CViamCartoGetPointCloudMapResponse{vcgpcmr: C.viam_carto_get_point_cloud_map_response{}}

	status := C.viam_carto_get_point_cloud_map(vc.viamCarto, &resp.vcgpcmr)

	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return nil, errors.New(string(status))
	}

	status = C.viam_carto_get_point_cloud_map_response_destroy(&resp.vcgpcmr)
	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return nil, errors.New(string(status))
	}

	return nil, nil
}

// CViamCartoGetInternalStateResponse is a struct to hold the c type viam_carto_get_internal_state_response
type CViamCartoGetInternalStateResponse struct {
	vcgisr C.viam_carto_get_internal_state_response
}

// GetInternalState is a wrapper for viam_carto_get_internal_state
func (vc *CViamCarto) GetInternalState(ctx context.Context) (func() ([]byte, error), error) {
	resp := CViamCartoGetInternalStateResponse{vcgisr: C.viam_carto_get_internal_state_response{}}

	status := C.viam_carto_get_internal_state(vc.viamCarto, &resp.vcgisr)

	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return nil, errors.New(string(status))
	}

	status = C.viam_carto_get_internal_state_response_destroy(&resp.vcgisr)
	if !successfulStatus(status) {
		// TODO: how to make this error useful?
		return nil, errors.New(string(status))
	}

	return nil, nil
}
