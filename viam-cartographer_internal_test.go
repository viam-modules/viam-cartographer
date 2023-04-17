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
	"go.viam.com/test"
	"google.golang.org/grpc"
	"google.golang.org/protobuf/types/known/structpb"

	inject "github.com/viamrobotics/viam-cartographer/internal/inject"
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

func makeQuaternionFromGenericMap(quat map[string]interface{}) spatialmath.Orientation {
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

	var inputPose commonv1.Pose
	var inputQuat map[string]interface{}
	var inputComponentRef string

	t.Run("successful client", func(t *testing.T) {
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
				makeQuaternionFromGenericMap(inputQuat),
			)
			test.That(t, pose, test.ShouldResemble, expectedPose)
			test.That(t, componentRef, test.ShouldEqual, inputComponentRef)
		})

		t.Run("non origin pose success", func(t *testing.T) {
			inputPose = commonv1.Pose{X: 5, Y: 5, Z: 5, OX: 0, OY: 0, OZ: 1, Theta: 0}
			inputQuat = map[string]interface{}{"real": 1.0, "imag": 1.0, "jmag": 0.0, "kmag": 0.0}
			inputComponentRef = "componentReference"
			pose, componentRef, err := svc.GetPosition(context.Background())
			test.That(t, err, test.ShouldBeNil)
			expectedPose := spatialmath.NewPose(
				r3.Vector{inputPose.X, inputPose.Y, inputPose.Z},
				makeQuaternionFromGenericMap(inputQuat),
			)
			test.That(t, pose, test.ShouldResemble, expectedPose)
			test.That(t, componentRef, test.ShouldEqual, inputComponentRef)
		})

		t.Run("empty compoent reference success", func(t *testing.T) {
			inputPose = commonv1.Pose{X: 6, Y: 5, Z: 5, OX: 0, OY: 0, OZ: 1, Theta: 0}
			inputQuat = map[string]interface{}{"real": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
			inputComponentRef = ""
			pose, componentRef, err := svc.GetPosition(context.Background())
			test.That(t, err, test.ShouldBeNil)
			expectedPose := spatialmath.NewPose(
				r3.Vector{inputPose.X, inputPose.Y, inputPose.Z},
				makeQuaternionFromGenericMap(inputQuat),
			)
			test.That(t, pose, test.ShouldResemble, expectedPose)
			test.That(t, componentRef, test.ShouldEqual, inputComponentRef)
		})

		t.Run("invalid quat map keys failure", func(t *testing.T) {
			inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
			inputQuat = map[string]interface{}{"invalid": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
			inputComponentRef = "componentReference"
			pose, componentRef, err := svc.GetPosition(context.Background())
			test.That(t, err, test.ShouldBeError)
			test.That(t, err.Error(), test.ShouldContainSubstring, "quaternion given, but invalid format detected")
			test.That(t, pose, test.ShouldBeNil)
			test.That(t, componentRef, test.ShouldEqual, "")
		})

		t.Run("invalid quat map values failure", func(t *testing.T) {
			inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
			inputQuat = map[string]interface{}{"real": "invalid", "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
			inputComponentRef = "componentReference"
			pose, componentRef, err := svc.GetPosition(context.Background())
			test.That(t, err, test.ShouldBeError)
			test.That(t, err.Error(), test.ShouldContainSubstring, "quaternion given, but invalid format detected")
			test.That(t, pose, test.ShouldBeNil)
			test.That(t, componentRef, test.ShouldEqual, "")
		})

		t.Run("empty quat map failure", func(t *testing.T) {
			inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
			inputQuat = map[string]interface{}{}
			inputComponentRef = "componentReference"
			pose, componentRef, err := svc.GetPosition(context.Background())
			test.That(t, err, test.ShouldBeError)
			test.That(t, err.Error(), test.ShouldContainSubstring, "quaternion given, but invalid format detected")
			test.That(t, pose, test.ShouldBeNil)
			test.That(t, componentRef, test.ShouldEqual, "")
		})
	})

	t.Run("invalid client", func(t *testing.T) {
		mockSLAMClient.GetPositionFunc = func(
			ctx context.Context,
			in *v1.GetPositionRequest,
			opts ...grpc.CallOption,
		) (*v1.GetPositionResponse, error) {
			resp, err := makeGetPositionResponse(&inputPose, &inputQuat, &inputComponentRef, "not_quat")
			return resp, err
		}

		t.Run("invalid client no quat key found in extra failure", func(t *testing.T) {
			inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
			inputQuat = map[string]interface{}{"real": 0.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
			inputComponentRef = "componentReference"
			pose, componentRef, err := svc.GetPosition(context.Background())
			test.That(t, err, test.ShouldBeError)
			test.That(t, err.Error(), test.ShouldContainSubstring, "quaternion not given")
			test.That(t, pose, test.ShouldBeNil)
			test.That(t, componentRef, test.ShouldEqual, "")
		})
	})

	t.Run("error client", func(t *testing.T) {
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
	})

	t.Run("nil client", func(t *testing.T) {
		mockSLAMClient.GetPositionFunc = nil

		t.Run("nil failure", func(t *testing.T) {
			pose, componentRef, err := svc.GetPosition(context.Background())
			test.That(t, err, test.ShouldBeError)
			test.That(t, err.Error(), test.ShouldContainSubstring, "no GetPositionFunc defined for injected SLAM service client")
			test.That(t, pose, test.ShouldBeNil)
			test.That(t, componentRef, test.ShouldEqual, "")
		})
	})
}

//nolint:dupl
func TestGetPointCloudMapEndpoint(t *testing.T) {
	svc := &cartographerService{}
	mockSLAMClient := &inject.SLAMServiceClient{}
	svc.clientAlgo = mockSLAMClient

	var chunkSizePointCloudMap int
	var inputPointCloudMapBytes []byte

	t.Run("successful client", func(t *testing.T) {
		mockSLAMClient.GetPointCloudMapFunc = func(
			ctx context.Context,
			in *v1.GetPointCloudMapRequest,
			opts ...grpc.CallOption,
		) (v1.SLAMService_GetPointCloudMapClient, error) {
			clientMock := makePointCloudClientMock(&inputPointCloudMapBytes, &chunkSizePointCloudMap)
			return clientMock, nil
		}

		t.Run("small chunk size success", func(t *testing.T) {
			chunkSizePointCloudMap = 1
			inputPointCloudMapBytes = []byte{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
			callback, err := svc.GetPointCloudMap(context.Background())
			test.That(t, err, test.ShouldBeNil)
			pointCloudMapBytes, err := slam.HelperConcatenateChunksToFull(callback)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, pointCloudMapBytes, test.ShouldResemble, inputPointCloudMapBytes)
		})

		t.Run("large chunk size success", func(t *testing.T) {
			chunkSizePointCloudMap = 100
			inputPointCloudMapBytes = []byte{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
			callback, err := svc.GetPointCloudMap(context.Background())
			test.That(t, err, test.ShouldBeNil)
			pointCloudMapBytes, err := slam.HelperConcatenateChunksToFull(callback)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, pointCloudMapBytes, test.ShouldResemble, inputPointCloudMapBytes)
		})

		t.Run("no bytes success", func(t *testing.T) {
			chunkSizePointCloudMap = 1
			inputPointCloudMapBytes = []byte{}
			callback, err := svc.GetPointCloudMap(context.Background())
			test.That(t, err, test.ShouldBeNil)
			pointCloudMapBytes, err := slam.HelperConcatenateChunksToFull(callback)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, pointCloudMapBytes, test.ShouldBeNil)
		})
	})

	t.Run("invalid client", func(t *testing.T) {
		mockSLAMClient.GetPointCloudMapFunc = func(
			ctx context.Context,
			in *v1.GetPointCloudMapRequest,
			opts ...grpc.CallOption,
		) (v1.SLAMService_GetPointCloudMapClient, error) {
			return nil, errors.New("error in get pointcloud map")
		}

		t.Run("invalid client error return failure", func(t *testing.T) {
			callback, err := svc.GetPointCloudMap(context.Background())
			test.That(t, err, test.ShouldBeError)
			test.That(t, err.Error(), test.ShouldContainSubstring, "error in get pointcloud map")
			test.That(t, callback, test.ShouldBeNil)
		})
	})
	t.Run("nil client", func(t *testing.T) {
		mockSLAMClient.GetPointCloudMapFunc = nil

		t.Run("nil failure", func(t *testing.T) {
			callback, err := svc.GetPointCloudMap(context.Background())
			test.That(t, err, test.ShouldBeError)
			test.That(t, err.Error(), test.ShouldContainSubstring, "no GetPointCloudMapFunc defined for injected SLAM service client")
			test.That(t, callback, test.ShouldBeNil)
		})
	})
}

//nolint:dupl
func TestGetInternalStateEndpoint(t *testing.T) {
	svc := &cartographerService{}
	mockSLAMClient := &inject.SLAMServiceClient{}
	svc.clientAlgo = mockSLAMClient

	var chunkSizeInternalState int
	var inputInternalStateBytes []byte

	t.Run("successful client", func(t *testing.T) {
		mockSLAMClient.GetInternalStateFunc = func(
			ctx context.Context,
			in *v1.GetInternalStateRequest,
			opts ...grpc.CallOption,
		) (v1.SLAMService_GetInternalStateClient, error) {
			clientMock := makeInternalStateClientMock(&inputInternalStateBytes, &chunkSizeInternalState)
			return clientMock, nil
		}

		t.Run("small internal state chunk size success", func(t *testing.T) {
			chunkSizeInternalState = 1
			inputInternalStateBytes = []byte{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
			callback, err := svc.GetInternalState(context.Background())
			test.That(t, err, test.ShouldBeNil)
			internalStateBytes, err := slam.HelperConcatenateChunksToFull(callback)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, internalStateBytes, test.ShouldResemble, inputInternalStateBytes)
		})

		t.Run("large internal state chunk size success", func(t *testing.T) {
			chunkSizeInternalState = 100
			inputInternalStateBytes = []byte{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
			callback, err := svc.GetInternalState(context.Background())
			test.That(t, err, test.ShouldBeNil)
			internalStateBytes, err := slam.HelperConcatenateChunksToFull(callback)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, internalStateBytes, test.ShouldResemble, inputInternalStateBytes)
		})

		t.Run("no internal state bytes success", func(t *testing.T) {
			chunkSizeInternalState = 1
			inputInternalStateBytes = []byte{}
			callback, err := svc.GetInternalState(context.Background())
			test.That(t, err, test.ShouldBeNil)
			internalStateBytes, err := slam.HelperConcatenateChunksToFull(callback)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, internalStateBytes, test.ShouldBeNil)
		})
	})

	t.Run("invalid client", func(t *testing.T) {
		mockSLAMClient.GetInternalStateFunc = func(
			ctx context.Context,
			in *v1.GetInternalStateRequest,
			opts ...grpc.CallOption,
		) (v1.SLAMService_GetInternalStateClient, error) {
			return nil, errors.New("error in get internal state")
		}

		t.Run("invalid client error return failure", func(t *testing.T) {
			callback, err := svc.GetInternalState(context.Background())
			test.That(t, err, test.ShouldBeError)
			test.That(t, err.Error(), test.ShouldContainSubstring, "error in get internal state")
			test.That(t, callback, test.ShouldBeNil)
		})
	})

	t.Run("nil client", func(t *testing.T) {
		mockSLAMClient.GetInternalStateFunc = nil

		t.Run("nil failure", func(t *testing.T) {
			callback, err := svc.GetInternalState(context.Background())
			test.That(t, err, test.ShouldBeError)
			test.That(t, err.Error(), test.ShouldContainSubstring, "no GetInternalState defined for injected SLAM service client")
			test.That(t, callback, test.ShouldBeNil)
		})
	})
}
