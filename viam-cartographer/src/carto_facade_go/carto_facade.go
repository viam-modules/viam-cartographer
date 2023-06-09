package carto_facade

/*
  #cgo LDFLAGS: -v -L../../build  -lviam-cartographer -lglog -lstdc++ -lpthread
	#include "../carto_facade/carto_facade.h"
*/
import "C"
import (
	"context"
	"errors"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

type CViamCarto struct {
	viam_carto *C.viam_carto
}

func GetCViamCartoObj() CViamCarto {
	return CViamCarto{viam_carto: &C.viam_carto{}}
}

type CViamCartoGetPositionResponse struct {
	vcgpr C.viam_carto_get_position_response
}

func (vc *CViamCarto) GetPosition(ctx context.Context) (spatialmath.Pose, string, error) {
	resp := CViamCartoGetPositionResponse{vcgpr: C.viam_carto_get_position_response{}}

	status := C.viam_carto_get_position(vc.viam_carto, &resp.vcgpr)

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

	status := C.viam_carto_get_point_cloud_map(vc.viam_carto, &resp.vcgpcmr)

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

	status := C.viam_carto_get_internal_state(vc.viam_carto, &resp.vcgisr)

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
