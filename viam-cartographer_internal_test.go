// Package viamcartographer implements simultaneous localization and mapping.
// This is an Experimental package.
package viamcartographer

import (
	"bytes"
	"context"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	commonv1 "go.viam.com/api/common/v1"
	v1 "go.viam.com/api/service/slam/v1"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
	"google.golang.org/grpc"
	"google.golang.org/protobuf/types/known/structpb"
)

var (
	chunkSizePointCloudMap     int
	chunkSizeInternalState     int
	inputComponentRef          string
	expectedPointCloudMapBytes []byte
	expectedInternalStateBytes []byte
	inputQuat                  map[string]interface{}
	inputPose                  commonv1.Pose
)

type pointCloudClientMock struct {
	grpc.ClientStream
	reader           *bytes.Reader
	clientBufferSize int
}

func makePointCloudClientMock(b *[]byte, clientBufSize *int) *pointCloudClientMock {
	return &pointCloudClientMock{reader: bytes.NewReader(*b), clientBufferSize: *clientBufSize}
}

func (m *pointCloudClientMock) Recv() (*v1.GetPointCloudMapResponse, error) {
	clientBuffer := make([]byte, m.clientBufferSize)
	n, err := m.reader.Read(clientBuffer)
	if err != nil {
		return nil, err
	}

	resp := &v1.GetPointCloudMapResponse{
		PointCloudPcdChunk: clientBuffer[:n],
	}
	return resp, nil
}

type internalStateClientMock struct {
	grpc.ClientStream
	reader           *bytes.Reader
	clientBufferSize int
}

func makeInternalStateClientMock(b *[]byte, clientBufSize *int) *internalStateClientMock {
	return &internalStateClientMock{reader: bytes.NewReader(*b), clientBufferSize: *clientBufSize}
}

func (m *internalStateClientMock) Recv() (*v1.GetInternalStateResponse, error) {
	clientBuffer := make([]byte, m.clientBufferSize)
	n, err := m.reader.Read(clientBuffer)
	if err != nil {
		return nil, err
	}

	resp := &v1.GetInternalStateResponse{
		InternalStateChunk: clientBuffer[:n],
	}
	return resp, nil
}

func makeGetPositionResponse(
	pose *commonv1.Pose,
	quat *map[string]interface{},
	componentReference *string,
	quatKey string,
) (*v1.GetPositionResponse, error) {
	extra, err := structpb.NewStruct(map[string]interface{}{quatKey: *quat})
	if err != nil {
		return nil, err
	}

	resp := &v1.GetPositionResponse{
		Pose:               pose,
		ComponentReference: *componentReference,
		Extra:              extra,
	}
	return resp, nil
}

func makeQuaternionFromProto(quat map[string]interface{}) spatialmath.Orientation {
	return &spatialmath.Quaternion{
		Real: quat["real"].(float64),
		Imag: quat["imag"].(float64),
		Jmag: quat["jmag"].(float64),
		Kmag: quat["kmag"].(float64),
	}
}

func TestGetPositionEndpoint(t *testing.T) {
	svc := &cartographerService{}
	mockSLAMClient := &inject.SLAMServiceClient{}
	svc.clientAlgo = mockSLAMClient

	// Add successful GetPosition client
	mockSLAMClient.GetPositionFunc = func(
		ctx context.Context,
		in *v1.GetPositionRequest,
		opts ...grpc.CallOption,
	) (*v1.GetPositionResponse, error) {
		resp, err := makeGetPositionResponse(&inputPose, &inputQuat, &inputComponentRef, "quat")
		return resp, err
	}

	t.Run("origin pose success", func(t *testing.T) {
		inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
		inputComponentRef = "componentReference"
		pose, componentRef, err := svc.GetPosition(context.Background())
		test.That(t, err, test.ShouldBeNil)
		expectedPose := spatialmath.NewPose(
			r3.Vector{inputPose.X, inputPose.Y, inputPose.Z},
			makeQuaternionFromProto(inputQuat),
		)
		test.That(t, pose, test.ShouldResemble, expectedPose)
		test.That(t, componentRef, test.ShouldEqual, inputComponentRef)
	})

	t.Run("non origin pose success", func(t *testing.T) {
		inputPose = commonv1.Pose{X: 5, Y: 5, Z: 5, OX: 1, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
		inputComponentRef = "componentReference"
		pose, componentRef, err := svc.GetPosition(context.Background())
		test.That(t, err, test.ShouldBeNil)
		expectedPose := spatialmath.NewPose(
			r3.Vector{inputPose.X, inputPose.Y, inputPose.Z},
			makeQuaternionFromProto(inputQuat),
		)
		test.That(t, pose, test.ShouldResemble, expectedPose)
		test.That(t, componentRef, test.ShouldEqual, inputComponentRef)
	})

	t.Run("empty component reference success", func(t *testing.T) {
		inputPose = commonv1.Pose{X: 5, Y: 5, Z: 5, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
		inputComponentRef = ""
		pose, componentRef, err := svc.GetPosition(context.Background())
		test.That(t, err, test.ShouldBeNil)
		expectedPose := spatialmath.NewPose(
			r3.Vector{inputPose.X, inputPose.Y, inputPose.Z},
			makeQuaternionFromProto(inputQuat),
		)
		test.That(t, pose, test.ShouldResemble, expectedPose)
		test.That(t, componentRef, test.ShouldEqual, inputComponentRef)
	})

	t.Run("invalid quat map keys failure", func(t *testing.T) {
		inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"r": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
		inputComponentRef = "componentReference"
		pose, componentRef, err := svc.GetPosition(context.Background())
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, "quaternion given, but invalid format detected")
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentRef, test.ShouldEqual, "")
	})

	t.Run("invalid quat map values failure", func(t *testing.T) {
		inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": "", "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
		inputComponentRef = "componentReference"
		pose, componentRef, err := svc.GetPosition(context.Background())
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, "quaternion given, but invalid format detected")
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentRef, test.ShouldEqual, "")
	})

	// Add invalid GetPosition client
	mockSLAMClient.GetPositionFunc = func(
		ctx context.Context,
		in *v1.GetPositionRequest,
		opts ...grpc.CallOption,
	) (*v1.GetPositionResponse, error) {
		resp, err := makeGetPositionResponse(&inputPose, &inputQuat, &inputComponentRef, "not_quat")
		return resp, err
	}

	t.Run("no quat key found in extra", func(t *testing.T) {
		inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 0.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
		inputComponentRef = "componentReference"
		pose, componentRef, err := svc.GetPosition(context.Background())
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, "quaternion not given")
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentRef, test.ShouldEqual, "")
	})

	// Add failing GetPosition client
	mockSLAMClient.GetPositionFunc = func(
		ctx context.Context,
		in *v1.GetPositionRequest,
		opts ...grpc.CallOption,
	) (*v1.GetPositionResponse, error) {
		return nil, errors.New("error in get position")
	}

	t.Run("error return failure", func(t *testing.T) {
		pose, componentRef, err := svc.GetPosition(context.Background())
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, "error in get position")
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentRef, test.ShouldEqual, "")
	})
}

//nolint:dupl
func TestGetPointCloudMapEndpoint(t *testing.T) {
	svc := &cartographerService{}
	mockSLAMClient := &inject.SLAMServiceClient{}
	svc.clientAlgo = mockSLAMClient

	// Add successful GetPointCloudMap client
	mockSLAMClient.GetPointCloudMapFunc = func(
		ctx context.Context,
		in *v1.GetPointCloudMapRequest,
		opts ...grpc.CallOption,
	) (v1.SLAMService_GetPointCloudMapClient, error) {
		clientMock := makePointCloudClientMock(&expectedPointCloudMapBytes, &chunkSizePointCloudMap)
		return clientMock, nil
	}

	t.Run("small chunk size success", func(t *testing.T) {
		chunkSizePointCloudMap = 1
		expectedPointCloudMapBytes = []byte{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
		callback, err := svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)
		pointCloudMapBytes, err := slam.HelperConcatenateChunksToFull(callback)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pointCloudMapBytes, test.ShouldResemble, expectedPointCloudMapBytes)
	})

	t.Run("large chunk size success", func(t *testing.T) {
		chunkSizePointCloudMap = 100
		expectedPointCloudMapBytes = []byte{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
		callback, err := svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)
		pointCloudMapBytes, err := slam.HelperConcatenateChunksToFull(callback)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pointCloudMapBytes, test.ShouldResemble, expectedPointCloudMapBytes)
	})

	t.Run("no bytes success", func(t *testing.T) {
		chunkSizePointCloudMap = 1
		expectedPointCloudMapBytes = []byte{}
		callback, err := svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeNil)
		pointCloudMapBytes, err := slam.HelperConcatenateChunksToFull(callback)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pointCloudMapBytes, test.ShouldBeNil)
	})

	// Add failing GetPointCloudMap client
	mockSLAMClient.GetPointCloudMapFunc = func(
		ctx context.Context,
		in *v1.GetPointCloudMapRequest,
		opts ...grpc.CallOption,
	) (v1.SLAMService_GetPointCloudMapClient, error) {
		return nil, errors.New("error in get pointcloud map")
	}

	t.Run("error return failure", func(t *testing.T) {
		callback, err := svc.GetPointCloudMap(context.Background())
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, "error in get pointcloud map")
		test.That(t, callback, test.ShouldBeNil)
	})
}

//nolint:dupl
func TestGetInternalStateEndpoint(t *testing.T) {
	svc := &cartographerService{}
	mockSLAMClient := &inject.SLAMServiceClient{}
	svc.clientAlgo = mockSLAMClient

	// Add successful GetInternalState client
	mockSLAMClient.GetInternalStateFunc = func(
		ctx context.Context,
		in *v1.GetInternalStateRequest,
		opts ...grpc.CallOption,
	) (v1.SLAMService_GetInternalStateClient, error) {
		clientMock := makeInternalStateClientMock(&expectedInternalStateBytes, &chunkSizeInternalState)
		return clientMock, nil
	}

	t.Run("small internal state chunk size success", func(t *testing.T) {
		chunkSizeInternalState = 1
		expectedInternalStateBytes = []byte{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
		callback, err := svc.GetInternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)
		internalStateBytes, err := slam.HelperConcatenateChunksToFull(callback)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalStateBytes, test.ShouldResemble, expectedInternalStateBytes)
	})

	t.Run("large internal state chunk size success", func(t *testing.T) {
		chunkSizeInternalState = 100
		expectedInternalStateBytes = []byte{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
		callback, err := svc.GetInternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)
		internalStateBytes, err := slam.HelperConcatenateChunksToFull(callback)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalStateBytes, test.ShouldResemble, expectedInternalStateBytes)
	})

	t.Run("no internal state bytes success", func(t *testing.T) {
		chunkSizeInternalState = 1
		expectedInternalStateBytes = []byte{}
		callback, err := svc.GetInternalState(context.Background())
		test.That(t, err, test.ShouldBeNil)
		internalStateBytes, err := slam.HelperConcatenateChunksToFull(callback)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, internalStateBytes, test.ShouldBeNil)
	})

	// Add failing GetInternalState client
	mockSLAMClient.GetInternalStateFunc = func(
		ctx context.Context,
		in *v1.GetInternalStateRequest,
		opts ...grpc.CallOption,
	) (v1.SLAMService_GetInternalStateClient, error) {
		return nil, errors.New("error in get internal state")
	}

	t.Run("error return failure", func(t *testing.T) {
		callback, err := svc.GetInternalState(context.Background())
		test.That(t, err, test.ShouldBeError)
		test.That(t, err.Error(), test.ShouldContainSubstring, "error in get internal state")
		test.That(t, callback, test.ShouldBeNil)
	})
}
