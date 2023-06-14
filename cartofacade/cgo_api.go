// Package cartofacade provides an api to call into c code
package cartofacade

/*
	#cgo CFLAGS: -I../viam-cartographer/src/carto_facade
  	#cgo LDFLAGS: -v -L../viam-cartographer/build  -lviam-cartographer -lglog -lstdc++ -lpthread

	#include "../viam-cartographer/src/carto_facade/carto_facade.h"
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
	"unsafe"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

func getErrorFromStatusCode(status C.int) error {
	switch int(status) {
	case C.VIAM_CARTO_SUCCESS:
		return nil
	case C.VIAM_CARTO_UNABLE_TO_AQUIRE_LOCK:
		return errors.New("error: VIAM_CARTO_UNABLE_TO_AQUIRE_LOCK")
	case C.VIAM_CARTO_VC_INVALID:
		return errors.New("error: VIAM_CARTO_VC_INVALID")
	case C.VIAM_CARTO_OUT_OF_MEMORY:
		return errors.New("error: VIAM_CARTO_OUT_OF_MEMORY")
	case C.VIAM_CARTO_DESTRUCTOR_ERROR:
		return errors.New("error: VIAM_CARTO_DESTRUCTOR_ERROR")
	case C.VIAM_CARTO_LIB_PLATFORM_INVALID:
		return errors.New("error: VIAM_CARTO_LIB_PLATFORM_INVALID")
	case C.VIAM_CARTO_LIB_INVALID:
		return errors.New("error: VIAM_CARTO_LIB_INVALID")
	case C.VIAM_CARTO_LIB_NOT_INITIALIZED:
		return errors.New("error: VIAM_CARTO_LIB_NOT_INITIALIZED")
	case C.VIAM_CARTO_SENSORS_LIST_EMPTY:
		return errors.New("error: VIAM_CARTO_SENSORS_LIST_EMPTY")
	case C.VIAM_CARTO_UNKNOWN_ERROR:
		return errors.New("error: VIAM_CARTO_UNKNOWN_ERROR")
	case C.VIAM_CARTO_DATA_DIR_NOT_PROVIDED:
		return errors.New("error: VIAM_CARTO_DATA_DIR_NOT_PROVIDED ")
	case C.VIAM_CARTO_SLAM_MODE_INVALID:
		return errors.New("error: VIAM_CARTO_SLAM_MODE_INVALID ")
	case C.VIAM_CARTO_LIDAR_CONFIG_INVALID:
		return errors.New("error: VIAM_CARTO_LIDAR_CONFIG_INVALID ")
	case C.VIAM_CARTO_MAP_RATE_SEC_INVALID:
		return errors.New("error: VIAM_CARTO_MAP_RATE_SEC_INVALID ")
	case C.VIAM_CARTO_COMPONENT_REFERENCE_INVALID:
		return errors.New("error: VIAM_CARTO_COMPONENT_REFERENCE_INVALID ")
	default:
		return errors.New("error: unknown status code")
	}
}

// CartoLib is a struct to hold the c type viam_carto_lib
type CartoLib struct {
	value *C.viam_carto_lib
}

// NewCartoLib calls viam_carto_lib_init and returns a pointer to a viam carto lib object.
func NewCartoLib(miniloglevel, verbose int) (CartoLib, error) {
	pVcl := C.alloc_viam_carto_lib()
	defer C.free_alloc_viam_carto_lib(pVcl)

	ppVcl := (**C.viam_carto_lib)(unsafe.Pointer(pVcl))

	status := C.viam_carto_lib_init(ppVcl, C.int(miniloglevel), C.int(verbose))
	if err := getErrorFromStatusCode(status); err != nil {
		return CartoLib{}, err
	}

	vcl := CartoLib{value: (*ppVcl)}

	return vcl, nil
}

// TerminateViamCartoLib calls viam_carto_lib_terminate to clean up memory for viam carto lib.
func (vcl *CartoLib) TerminateViamCartoLib() error {
	status := C.viam_carto_lib_terminate((**C.viam_carto_lib)(unsafe.Pointer(vcl)))
	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}
	return nil
}

// CViamCarto is a struct to hold the c type viam_carto
type Carto struct {
	value *C.viam_carto
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
func NewViamCarto(vcl CartoLib) (Carto, error) {
	// TODO: get vcl as a param & initialize c queue channel here
	pVc := C.alloc_viam_carto()
	defer C.free_alloc_viam_carto(pVc)

	ppVc := (**C.viam_carto)(unsafe.Pointer(pVc))

	// TODO: Init with data
	vcc := getConfig()
	defer cleanConfig(vcc)
	vcac := C.viam_carto_algo_config{}

	status := C.viam_carto_init(ppVc, vcl.value, vcc, vcac)
	if err := getErrorFromStatusCode(status); err != nil {
		return Carto{}, err
	}

	vc := Carto{value: (*ppVc)}
	return vc, nil
}

// Start is a wrapper for viam_carto_start
func (vc *Carto) Start(ctx context.Context) error {
	status := C.viam_carto_start(vc.value)

	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}

	return nil
}

// Stop is a wrapper for viam_carto_stop
func (vc *Carto) Stop(ctx context.Context) error {
	status := C.viam_carto_stop(vc.value)

	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}

	return nil
}

// TerminateViamCarto calls viam_carto_terminate to clean up memory for viam carto
func (vc *Carto) TerminateViamCarto(ctx context.Context) error {
	status := C.viam_carto_terminate(&vc.value)

	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}

	return nil
}

// CViamCartoAddSensorReading is a struct to hold the c type viam_carto_sensor_reading
type CViamCartoAddSensorReading struct {
	vcasr C.viam_carto_sensor_reading
}

// AddSensorReading is a wrapper for viam_carto_add_sensor_reading
func (vc *Carto) AddSensorReading(ctx context.Context) error {
	resp := CViamCartoAddSensorReading{vcasr: C.viam_carto_sensor_reading{}}

	status := C.viam_carto_add_sensor_reading(vc.value, &resp.vcasr)

	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}

	status = C.viam_carto_add_sensor_reading_destroy(&resp.vcasr)
	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}

	return nil
}

// CViamCartoGetPositionResponse is a struct to hold the c type viam_carto_get_position_response
type CViamCartoGetPositionResponse struct {
	vcgpr C.viam_carto_get_position_response
}

// GetPosition is a wrapper for viam_carto_get_position
func (vc *Carto) GetPosition(ctx context.Context) (spatialmath.Pose, string, error) {
	resp := CViamCartoGetPositionResponse{vcgpr: C.viam_carto_get_position_response{}}

	status := C.viam_carto_get_position(vc.value, &resp.vcgpr)

	if err := getErrorFromStatusCode(status); err != nil {
		// TODO: how to make this error useful?
		return nil, "", err
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
	if err := getErrorFromStatusCode(status); err != nil {
		// TODO: how to make this error useful?
		return nil, "", err
	}

	// TODO: fill component reference in?
	return spatialmath.NewPose(pos, ori), "", nil
}

// CViamCartoGetPointCloudMapResponse is a struct to hold the c type viam_carto_get_point_cloud_map_response
type CViamCartoGetPointCloudMapResponse struct {
	vcgpcmr C.viam_carto_get_point_cloud_map_response
}

// GetPointCloudMap is a wrapper for viam_carto_get_point_cloud_map
func (vc *Carto) GetPointCloudMap(ctx context.Context) ([]byte, error) {
	// TODO: determine whether or not return needs to be a pointer for performance reasons
	resp := CViamCartoGetPointCloudMapResponse{vcgpcmr: C.viam_carto_get_point_cloud_map_response{}}

	status := C.viam_carto_get_point_cloud_map(vc.value, &resp.vcgpcmr)

	if err := getErrorFromStatusCode(status); err != nil {
		return nil, err
	}

	status = C.viam_carto_get_point_cloud_map_response_destroy(&resp.vcgpcmr)
	if err := getErrorFromStatusCode(status); err != nil {
		return nil, err
	}

	return nil, errors.New("unimplemented error")
}

// CViamCartoGetInternalStateResponse is a struct to hold the c type viam_carto_get_internal_state_response
type CViamCartoGetInternalStateResponse struct {
	vcgisr C.viam_carto_get_internal_state_response
}

// GetInternalState is a wrapper for viam_carto_get_internal_state
func (vc *Carto) GetInternalState(ctx context.Context) ([]byte, error) {
	resp := CViamCartoGetInternalStateResponse{vcgisr: C.viam_carto_get_internal_state_response{}}

	status := C.viam_carto_get_internal_state(vc.value, &resp.vcgisr)

	if err := getErrorFromStatusCode(status); err != nil {
		return nil, err
	}

	status = C.viam_carto_get_internal_state_response_destroy(&resp.vcgisr)
	if err := getErrorFromStatusCode(status); err != nil {
		return nil, err
	}

	return nil, errors.New("unimplemented error")
}
