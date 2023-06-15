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
	"time"
	"unsafe"
)

// CartoLib holds the c type viam_carto_lib
type CartoLib struct {
	value *C.viam_carto_lib
}

// Carto holds the c type viam_carto
type Carto struct {
	value *C.viam_carto
}

// AddSensorReading holds the c type viam_carto_sensor_reading
type AddSensorReading struct {
	value C.viam_carto_sensor_reading
}

// GetPositionResponse holds the c type viam_carto_get_position_response
type GetPositionResponse struct {
	value C.viam_carto_get_position_response
}

// GetPositionTempHolder holds values returned from c to be processed later
type GetPositionHolder struct {
	x float64
	y float64
	z float64

	ox    float64
	oy    float64
	oz    float64
	theta float64

	real float64
	imag float64
	jmag float64
	kmag float64

	compReference string
}

// GetInternalStateResponse holds the c type viam_carto_get_internal_state_response
type GetInternalStateResponse struct {
	value C.viam_carto_get_internal_state_response
}

type LidarConfig int64

const (
	twoD LidarConfig = iota
	threeD
)

type Mode int64

const (
	localizing Mode = iota
	mapping
	updating
)

type CartoConfig struct {
	sensors            []string
	mapRateSecond      int
	dataDir            string
	componentReference string
	mode               Mode
	lidarConfig        LidarConfig
}

// NewCartoLib calls viam_carto_lib_init and returns a pointer to a viam carto lib object.
func NewCartoLib(miniloglevel, verbose int) (CartoLib, error) {
	pVcl := C.alloc_viam_carto_lib()
	defer C.free_alloc_viam_carto_lib(pVcl)

	ppVcl := (**C.viam_carto_lib)(unsafe.Pointer(&pVcl))
	/*
		todo: what is going on here? are we not getting correct amount of indirection?
					fmt.Printf("%#v\n", pVcl) = &cartofacade._Ctype_struct_viam_carto_lib{minloglevel:0, verbose:0}
					fmt.Println(&pVcl) = 0x14000128078
					fmt.Println(goPtr) = 0x600001ee8000
					// ppVcl := (**C.viam_carto_lib)(goPtr)
					fmt.Println(ppVcl) = 0x600001ee8000
	*/

	status := C.viam_carto_lib_init(ppVcl, C.int(miniloglevel), C.int(verbose))
	if err := getErrorFromStatusCode(status); err != nil {
		return CartoLib{}, err
	}

	vcl := CartoLib{value: (*ppVcl)}

	return vcl, nil
}

// TerminateCartoLib calls viam_carto_lib_terminate to clean up memory for viam carto lib.
func (vcl *CartoLib) TerminateCartoLib() error {
	status := C.viam_carto_lib_terminate((**C.viam_carto_lib)(unsafe.Pointer(vcl)))
	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}
	return nil
}

// NewViamCarto calls viam_carto_init and returns a pointer to a viam carto object.
func NewCarto(cfg CartoConfig, vcl CartoLib) (Carto, error) {
	pVc := C.alloc_viam_carto()
	defer C.free_alloc_viam_carto(pVc)

	ppVc := (**C.viam_carto)(unsafe.Pointer(&pVc))

	vcc := getConfig(cfg)
	defer C.free_bstring_array(vcc.sensors)

	// TODO: what needs to go in here?
	vcac := C.viam_carto_algo_config{}

	status := C.viam_carto_init(ppVc, vcl.value, vcc, vcac)
	if err := getErrorFromStatusCode(status); err != nil {
		return Carto{}, err
	}

	return Carto{value: (*ppVc)}, nil
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
func (vc *Carto) TerminateCarto(ctx context.Context) error {
	status := C.viam_carto_terminate(&vc.value)

	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}

	return nil
}

// AddSensorReading is a wrapper for viam_carto_add_sensor_reading
func (vc *Carto) AddSensorReading(ctx context.Context, readings []byte, timestamp time.Time) error {
	sensorReading := C.viam_carto_sensor_reading{}
	sensorReading.sensor_reading = C.blk2bstr(unsafe.Pointer(&readings[0]), C.int(len(readings)))

	sensorReading.sensor_reading_time_unix_micro = _Ctype_ulonglong(timestamp.UnixMicro())

	resp := AddSensorReading{value: sensorReading}

	status := C.viam_carto_add_sensor_reading(vc.value, &resp.value)

	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}

	status = C.viam_carto_add_sensor_reading_destroy(&resp.value)
	if err := getErrorFromStatusCode(status); err != nil {
		return err
	}

	return nil
}

// GetPosition is a wrapper for viam_carto_get_position
func (vc *Carto) GetPosition(ctx context.Context) (GetPositionHolder, error) {
	resp := GetPositionResponse{value: C.viam_carto_get_position_response{}}

	status := C.viam_carto_get_position(vc.value, &resp.value)

	if err := getErrorFromStatusCode(status); err != nil {
		return GetPositionHolder{}, err
	}

	getPositionHolder := GetPositionHolder{
		x: float64(resp.value.x),
		y: float64(resp.value.y),
		z: float64(resp.value.z),

		ox:    float64(resp.value.o_x),
		oy:    float64(resp.value.o_y),
		oz:    float64(resp.value.o_z),
		theta: float64(resp.value.theta),

		real: float64(resp.value.real),
		imag: float64(resp.value.imag),
		jmag: float64(resp.value.jmag),
		kmag: float64(resp.value.kmag),

		compReference: bstringToGoString(resp.value.component_reference),
	}

	status = C.viam_carto_get_position_response_destroy(&resp.value)
	if err := getErrorFromStatusCode(status); err != nil {
		return GetPositionHolder{}, err
	}

	return getPositionHolder, nil
}

// GetPointCloudMapResponse is a struct to hold the c type viam_carto_get_point_cloud_map_response
type GetPointCloudMapResponse struct {
	value C.viam_carto_get_point_cloud_map_response
}

// GetPointCloudMap is a wrapper for viam_carto_get_point_cloud_map
func (vc *Carto) GetPointCloudMap(ctx context.Context) ([]byte, error) {
	// TODO: determine whether or not return needs to be a pointer for performance reasons
	resp := GetPointCloudMapResponse{value: C.viam_carto_get_point_cloud_map_response{}}

	status := C.viam_carto_get_point_cloud_map(vc.value, &resp.value)

	if err := getErrorFromStatusCode(status); err != nil {
		return nil, err
	}

	status = C.viam_carto_get_point_cloud_map_response_destroy(&resp.value)
	if err := getErrorFromStatusCode(status); err != nil {
		return nil, err
	}

	bytes := (*[]byte)(unsafe.Pointer(&resp.value.point_cloud_pcd))
	return *bytes, nil
}

// GetInternalState is a wrapper for viam_carto_get_internal_state
func (vc *Carto) GetInternalState(ctx context.Context) ([]byte, error) {
	resp := GetInternalStateResponse{value: C.viam_carto_get_internal_state_response{}}

	status := C.viam_carto_get_internal_state(vc.value, &resp.value)

	if err := getErrorFromStatusCode(status); err != nil {
		return nil, err
	}

	status = C.viam_carto_get_internal_state_response_destroy(&resp.value)
	if err := getErrorFromStatusCode(status); err != nil {
		return nil, err
	}

	bytes := (*[]byte)(unsafe.Pointer(&resp.value.internal_state))
	return *bytes, nil
}

func bStringToGoStringSlice(bstr *C.bstring, length int) []string {
	bStrings := (*[1 << 28]C.bstring)(unsafe.Pointer(bstr))[:length:length]

	goStrings := []string{}
	for _, bString := range bStrings {
		goStrings = append(goStrings, C.GoString(C.bstr2cstr(bString, 0)))
	}
	return goStrings
}

func bstringToGoString(bstr C.bstring) string {
	return C.GoString(C.bstr2cstr(bstr, 0))
}

func getLidarConfig(lidarConfig LidarConfig) C.viam_carto_LIDAR_CONFIG {
	switch lidarConfig {
	case twoD:
		return C.VIAM_CARTO_TWO_D
	case threeD:
		return C.VIAM_CARTO_THREE_D
	}
	// TODO: some error case?
	return 0
}

func getConfig(cfg CartoConfig) C.viam_carto_config {
	vcc := C.viam_carto_config{}

	// create pointer to bstring which can represent a list of sensors
	sz := len(cfg.sensors)
	pSensor := C.alloc_bstring_array(C.size_t(sz))
	sensorSlice := unsafe.Slice(pSensor, sz)
	for i, sensor := range cfg.sensors {
		sensorSlice[i] = C.bfromcstr(C.CString(sensor))
	}

	vcc.sensors = pSensor
	vcc.sensors_len = C.int(sz)
	vcc.map_rate_sec = C.int(cfg.mapRateSecond)
	vcc.data_dir = C.bfromcstr(C.CString(cfg.dataDir))
	vcc.component_reference = C.bfromcstr(C.CString(cfg.componentReference))
	vcc.lidar_config = getLidarConfig(cfg.lidarConfig)

	return vcc
}

// FreeBstringArray used to cleanup a bstring array (needs to be a go func so it can be used in tests)
func FreeBstringArray(arr *C.bstring) {
	C.free_bstring_array(arr)
}

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
