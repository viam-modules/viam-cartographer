// Package cartofacade provides an api to call into c code
//
//nolint:lll
package cartofacade

/*
	//TODO: check if removing homebrew prefix in cmake causes link to fail
	#cgo CFLAGS: -I../viam-cartographer/src/carto_facade

	// the libraries that need to be linked can be dermined by checking line 258 of the build.ninja file that is autogenerated during make build
	#cgo LDFLAGS: -L../viam-cartographer/build -L../viam-cartographer/cartographer/build -lviam-cartographer  -lcartographer -ldl -lm -labsl_hash  -labsl_city -labsl_bad_optional_access -labsl_strerror  -labsl_str_format_internal -labsl_synchronization -labsl_strings -labsl_throw_delegate -lcairo -llua5.3 -lstdc++ -lceres -lprotobuf -lglog -lboost_filesystem -lboost_iostreams -lpcl_io -lpcl_common -labsl_raw_hash_set

	#include "../viam-cartographer/src/carto_facade/carto_facade.h"

	viam_carto_lib* alloc_viam_carto_lib() { return (viam_carto_lib*) malloc(sizeof(viam_carto_lib)); }
	void free_viam_carto_lib(viam_carto_lib* p) { free(p); }

	viam_carto* alloc_viam_carto() { return (viam_carto*) malloc(sizeof(viam_carto)); }
	void free_viam_carto(viam_carto* p) { free(p); }

	bstring* alloc_bstring_array(size_t len) { return (bstring*) malloc(len * sizeof(bstring)); }
	int free_bstring_array(bstring* p, size_t len) {
		int result;
		for (size_t i = 0; i < len; i++) {
			result = bdestroy(p[i]);
			if (result != BSTR_OK) {
				return result;
			};
		}
		free(p);
		return BSTR_OK;
	}
*/
import "C"

import (
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

// GetPosition holds values returned from c to be processed later
type GetPosition struct {
	X float64
	Y float64
	Z float64

	Ox    float64
	Oy    float64
	Oz    float64
	Theta float64

	Real float64
	Imag float64
	Jmag float64
	Kmag float64

	ComponentReference string
}

// LidarConfig represents the lidar configuration
type LidarConfig int64

const (
	twoD LidarConfig = iota
	threeD
)

// CartoConfig contains config values from app
type CartoConfig struct {
	Sensors            []string
	MapRateSecond      int
	DataDir            string
	ComponentReference string
	LidarConfig        LidarConfig
}

// CartoAlgoConfig contains config values from app
type CartoAlgoConfig struct {
	optimizeOnStart      bool
	optimizeEveryNNodes  int
	numRangeData         int
	missingDataRayLength float32
	maxRange             float32
	minRange             float32
	maxSubmapsToKeep     int
	freshSubmapsCount    int
	minCoveredArea       float64
	minAddedSubmapsCount int
	occupiedSpaceWeight  float64
	translationWeight    float64
	rotationWeight       float64
}

// NewLib calls viam_carto_lib_init and returns a pointer to a viam carto lib object.
func NewLib(miniloglevel, verbose int) (CartoLib, error) {
	pVcl := C.alloc_viam_carto_lib()
	if pVcl == nil {
		return CartoLib{}, errors.New("unable to allocate memory for viam carto lib")
	}
	defer C.free_viam_carto_lib(pVcl)

	ppVcl := (**C.viam_carto_lib)(unsafe.Pointer(&pVcl))
	/*
		Question: what is going on here? are we not getting correct amount of indirection?
					fmt.Printf("%#v\n", pVcl) = &cartofacade._Ctype_struct_viam_carto_lib{minloglevel:0, verbose:0}
					fmt.Println(&pVcl) = 0x14000128078
					fmt.Println(goPtr) = 0x600001ee8000
					// ppVcl := (**C.viam_carto_lib)(goPtr)
					fmt.Println(ppVcl) = 0x600001ee8000
	*/

	status := C.viam_carto_lib_init(ppVcl, C.int(miniloglevel), C.int(verbose))
	if err := toError(status); err != nil {
		return CartoLib{}, err
	}

	vcl := CartoLib{value: (*ppVcl)}

	return vcl, nil
}

// Terminate calls viam_carto_lib_terminate to clean up memory for viam carto lib.
func (vcl *CartoLib) Terminate() error {
	status := C.viam_carto_lib_terminate(&vcl.value)
	if err := toError(status); err != nil {
		return err
	}
	return nil
}

// New calls viam_carto_init and returns a pointer to a viam carto object.
func New(cfg CartoConfig, acfg CartoAlgoConfig, vcl CartoLib) (Carto, error) {
	pVc := C.alloc_viam_carto()
	if pVc == nil {
		return Carto{}, errors.New("unable to allocate memory for viam carto")
	}
	defer C.free_viam_carto(pVc)

	ppVc := (**C.viam_carto)(unsafe.Pointer(&pVc))

	vcc, err := getConfig(cfg)
	if err != nil {
		return Carto{}, err
	}

	vcac := toAlgoConfig(acfg)

	status := C.viam_carto_init(ppVc, vcl.value, vcc, vcac)
	if err := toError(status); err != nil {
		return Carto{}, err
	}

	status = C.free_bstring_array(vcc.sensors, C.size_t(len(cfg.Sensors)))
	if status != C.BSTR_OK {
		return Carto{}, errors.New("unable to free memory for sensor list")
	}
	return Carto{value: (*ppVc)}, nil
}

// Start is a wrapper for viam_carto_start
func (vc *Carto) Start() error {
	status := C.viam_carto_start(vc.value)

	if err := toError(status); err != nil {
		return err
	}

	return nil
}

// Stop is a wrapper for viam_carto_stop
func (vc *Carto) Stop() error {
	status := C.viam_carto_stop(vc.value)

	if err := toError(status); err != nil {
		return err
	}

	return nil
}

// Terminate calls viam_carto_terminate to clean up memory for viam carto
func (vc *Carto) Terminate() error {
	status := C.viam_carto_terminate(&vc.value)

	if err := toError(status); err != nil {
		return err
	}

	return nil
}

// AddSensorReading is a wrapper for viam_carto_add_sensor_reading
func (vc *Carto) AddSensorReading(readings []byte, timestamp time.Time) error {
	value := toSensorReading(readings, timestamp)

	status := C.viam_carto_add_sensor_reading(vc.value, &value)

	if err := toError(status); err != nil {
		return err
	}

	status = C.viam_carto_add_sensor_reading_destroy(&value)
	if err := toError(status); err != nil {
		return err
	}

	return nil
}

// GetPosition is a wrapper for viam_carto_get_position
func (vc *Carto) GetPosition() (GetPosition, error) {
	value := C.viam_carto_get_position_response{}

	status := C.viam_carto_get_position(vc.value, &value)

	if err := toError(status); err != nil {
		return GetPosition{}, err
	}

	getPosition := toGetPositionResponse(value)

	status = C.viam_carto_get_position_response_destroy(&value)
	if err := toError(status); err != nil {
		return GetPosition{}, err
	}

	return getPosition, nil
}

// GetPointCloudMap is a wrapper for viam_carto_get_point_cloud_map
func (vc *Carto) GetPointCloudMap() ([]byte, error) {
	// TODO: determine whether or not return needs to be a pointer for performance reasons
	value := C.viam_carto_get_point_cloud_map_response{}

	status := C.viam_carto_get_point_cloud_map(vc.value, &value)

	if err := toError(status); err != nil {
		return nil, err
	}

	status = C.viam_carto_get_point_cloud_map_response_destroy(&value)
	if err := toError(status); err != nil {
		return nil, err
	}

	if value.point_cloud_pcd != nil {
		return bstringToByteSlice(value.point_cloud_pcd), nil
	}
	return nil, errors.New("nil pointcloud")
}

// GetInternalState is a wrapper for viam_carto_get_internal_state
func (vc *Carto) GetInternalState() ([]byte, error) {
	value := C.viam_carto_get_internal_state_response{}

	status := C.viam_carto_get_internal_state(vc.value, &value)

	if err := toError(status); err != nil {
		return nil, err
	}

	status = C.viam_carto_get_internal_state_response_destroy(&value)
	if err := toError(status); err != nil {
		return nil, err
	}

	if value.internal_state != nil {
		return bstringToByteSlice(value.internal_state), nil
	}
	return nil, errors.New("nil internal state")
}

// this function is only used for testing purposes, but needs to be in this file as CGo is not supported in go test files
func bStringToGoStringSlice(bstr *C.bstring, length int) []string {
	bStrings := (*[1 << 28]C.bstring)(unsafe.Pointer(bstr))[:length:length]

	goStrings := []string{}
	for _, bString := range bStrings {
		goStrings = append(goStrings, bstringToGoString(bString))
	}
	return goStrings
}

// this function is only used for testing purposes, but needs to be in this file as CGo is not supported in go test files
func getTestGetPositionResponse() C.viam_carto_get_position_response {
	gpr := C.viam_carto_get_position_response{}

	gpr.x = C.double(100)
	gpr.y = C.double(200)
	gpr.z = C.double(300)

	gpr.o_x = C.double(400)
	gpr.o_y = C.double(500)
	gpr.o_z = C.double(600)

	gpr.imag = C.double(700)
	gpr.jmag = C.double(800)
	gpr.kmag = C.double(900)

	gpr.theta = C.double(1000)
	gpr.real = C.double(1100)

	gpr.component_reference = goStringToBstring("C++ component reference")

	return gpr
}

func bstringToGoString(bstr C.bstring) string {
	return C.GoStringN(C.bstr2cstr(bstr, 0), bstr.slen)
}

func goStringToBstring(goStr string) C.bstring {
	return C.blk2bstr(unsafe.Pointer(C.CString(goStr)), C.int(len(goStr)))
}

func toLidarConfig(lidarConfig LidarConfig) (C.viam_carto_LIDAR_CONFIG, error) {
	switch lidarConfig {
	case twoD:
		return C.VIAM_CARTO_TWO_D, nil
	case threeD:
		return C.VIAM_CARTO_THREE_D, nil
	default:
		return 0, errors.New("invalid lidar config value")
	}
}

func getConfig(cfg CartoConfig) (C.viam_carto_config, error) {
	vcc := C.viam_carto_config{}

	// create pointer to bstring which can represent a list of sensors
	sz := len(cfg.Sensors)
	pSensor := C.alloc_bstring_array(C.size_t(sz))
	if pSensor == nil {
		return C.viam_carto_config{}, errors.New("unable to allocate memory for sensor list")
	}
	sensorSlice := unsafe.Slice(pSensor, sz)
	for i, sensor := range cfg.Sensors {
		sensorSlice[i] = goStringToBstring(sensor)
	}
	lidarCfg, err := toLidarConfig(cfg.LidarConfig)
	if err != nil {
		return C.viam_carto_config{}, err
	}

	vcc.sensors = pSensor
	vcc.sensors_len = C.int(sz)
	vcc.map_rate_sec = C.int(cfg.MapRateSecond)
	vcc.data_dir = goStringToBstring(cfg.DataDir)
	vcc.component_reference = goStringToBstring(cfg.ComponentReference)
	vcc.lidar_config = lidarCfg

	return vcc, nil
}

func toAlgoConfig(acfg CartoAlgoConfig) C.viam_carto_algo_config {
	vcac := C.viam_carto_algo_config{}
	vcac.optimize_on_start = C.bool(acfg.optimizeOnStart)
	vcac.optimize_every_n_nodes = C.int(acfg.optimizeEveryNNodes)
	vcac.num_range_data = C.int(acfg.numRangeData)
	vcac.missing_data_ray_length = C.float(acfg.missingDataRayLength)
	vcac.max_range = C.float(acfg.maxRange)
	vcac.min_range = C.float(acfg.minRange)
	vcac.max_submaps_to_keep = C.int(acfg.maxSubmapsToKeep)
	vcac.fresh_submaps_count = C.int(acfg.freshSubmapsCount)
	vcac.min_covered_area = C.double(acfg.minCoveredArea)
	vcac.min_added_submaps_count = C.int(acfg.minAddedSubmapsCount)
	vcac.occupied_space_weight = C.double(acfg.occupiedSpaceWeight)
	vcac.translation_weight = C.double(acfg.translationWeight)
	vcac.rotation_weight = C.double(acfg.rotationWeight)
	return vcac
}

func toGetPositionResponse(value C.viam_carto_get_position_response) GetPosition {
	return GetPosition{
		X: float64(value.x),
		Y: float64(value.y),
		Z: float64(value.z),

		Ox:    float64(value.o_x),
		Oy:    float64(value.o_y),
		Oz:    float64(value.o_z),
		Theta: float64(value.theta),

		Real: float64(value.real),
		Imag: float64(value.imag),
		Jmag: float64(value.jmag),
		Kmag: float64(value.kmag),

		ComponentReference: bstringToGoString(value.component_reference),
	}
}

func toSensorReading(readings []byte, timestamp time.Time) C.viam_carto_sensor_reading {
	sr := C.viam_carto_sensor_reading{}
	sr.sensor_reading = C.blk2bstr(unsafe.Pointer(&readings[0]), C.int(len(readings)))
	sr.sensor_reading_time_unix_micro = C.ulonglong(timestamp.UnixMicro())
	return sr
}

func bstringToByteSlice(bstr C.bstring) []byte {
	return C.GoBytes(unsafe.Pointer(bstr.data), bstr.slen)
}

// freeBstringArray used to cleanup a bstring array (needs to be a go func so it can be used in tests)
func freeBstringArray(arr *C.bstring, length C.int) {
	lengthSizeT := C.size_t(length)
	C.free_bstring_array(arr, lengthSizeT)
}

func toError(status C.int) error {
	switch int(status) {
	case C.VIAM_CARTO_SUCCESS:
		return nil
	case C.VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK:
		return errors.New("VIAM_CARTO_UNABLE_TO_ACQUIRE_LOCK")
	case C.VIAM_CARTO_VC_INVALID:
		return errors.New("VIAM_CARTO_VC_INVALID")
	case C.VIAM_CARTO_OUT_OF_MEMORY:
		return errors.New("VIAM_CARTO_OUT_OF_MEMORY")
	case C.VIAM_CARTO_DESTRUCTOR_ERROR:
		return errors.New("VIAM_CARTO_DESTRUCTOR_ERROR")
	case C.VIAM_CARTO_LIB_PLATFORM_INVALID:
		return errors.New("VIAM_CARTO_LIB_PLATFORM_INVALID")
	case C.VIAM_CARTO_LIB_INVALID:
		return errors.New("VIAM_CARTO_LIB_INVALID")
	case C.VIAM_CARTO_LIB_NOT_INITIALIZED:
		return errors.New("VIAM_CARTO_LIB_NOT_INITIALIZED")
	case C.VIAM_CARTO_SENSORS_LIST_EMPTY:
		return errors.New("VIAM_CARTO_SENSORS_LIST_EMPTY")
	case C.VIAM_CARTO_UNKNOWN_ERROR:
		return errors.New("VIAM_CARTO_UNKNOWN_ERROR")
	case C.VIAM_CARTO_DATA_DIR_NOT_PROVIDED:
		return errors.New("VIAM_CARTO_DATA_DIR_NOT_PROVIDED")
	case C.VIAM_CARTO_SLAM_MODE_INVALID:
		return errors.New("VIAM_CARTO_SLAM_MODE_INVALID")
	case C.VIAM_CARTO_LIDAR_CONFIG_INVALID:
		return errors.New("VIAM_CARTO_LIDAR_CONFIG_INVALID")
	case C.VIAM_CARTO_MAP_RATE_SEC_INVALID:
		return errors.New("VIAM_CARTO_MAP_RATE_SEC_INVALID")
	case C.VIAM_CARTO_COMPONENT_REFERENCE_INVALID:
		return errors.New("VIAM_CARTO_COMPONENT_REFERENCE_INVALID")
	default:
		return errors.New("status code unclassified")
	}
}
