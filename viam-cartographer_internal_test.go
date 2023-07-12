package viamcartographer

import (
	"bytes"
	"context"
	"os"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	commonv1 "go.viam.com/api/common/v1"
	v1 "go.viam.com/api/service/slam/v1"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"
	"google.golang.org/grpc"
	"google.golang.org/protobuf/types/known/structpb"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
	inject "github.com/viamrobotics/viam-cartographer/testhelper/inject"
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
	svc := &cartographerService{Named: resource.NewName(slam.API, "test").AsNamed()}
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
				r3.Vector{X: inputPose.X, Y: inputPose.Y, Z: inputPose.Z},
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
				r3.Vector{X: inputPose.X, Y: inputPose.Y, Z: inputPose.Z},
				makeQuaternionFromGenericMap(inputQuat),
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
				r3.Vector{X: inputPose.X, Y: inputPose.Y, Z: inputPose.Z},
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

func setMockGetPositionFunc(
	mockCartoFacade *cartofacade.Mock,
	pos cartofacade.GetPosition,
) {
	mockCartoFacade.GetPositionFunc = func(
		ctx context.Context,
		timeout time.Duration,
	) (cartofacade.GetPosition, error) {
		return pos,
			nil
	}
}

func checkGetPositionModularizationV2Outputs(
	t *testing.T,
	mockCartoFacade *cartofacade.Mock,
	inputPose commonv1.Pose,
	inputQuat map[string]interface{},
	inputComponentRef string,
	svc *cartographerService,
	pos cartofacade.GetPosition,
) {
	setMockGetPositionFunc(mockCartoFacade, pos)

	pose, componentReference, err := svc.GetPosition(context.Background())
	test.That(t, err, test.ShouldBeNil)
	expectedPose := spatialmath.NewPose(
		r3.Vector{X: inputPose.X, Y: inputPose.Y, Z: inputPose.Z},
		makeQuaternionFromGenericMap(inputQuat),
	)
	test.That(t, pose, test.ShouldResemble, expectedPose)
	test.That(t, componentReference, test.ShouldEqual, inputComponentRef)
}

func TestGetPositionModularizationV2Endpoint(t *testing.T) {
	svc := &cartographerService{Named: resource.NewName(slam.API, "test").AsNamed()}
	mockCartoFacade := &cartofacade.Mock{}
	svc.cartofacade = mockCartoFacade
	svc.modularizationV2Enabled = true

	originPos := cartofacade.GetPosition{
		X:    0,
		Y:    0,
		Z:    0,
		Real: 1.0,
		Imag: 0.0,
		Jmag: 0.0,
		Kmag: 0.0,
	}

	nonOriginPos := cartofacade.GetPosition{
		X:    5,
		Y:    5,
		Z:    5,
		Real: 1.0,
		Imag: 1.0,
		Jmag: 0.0,
		Kmag: 0.0,
	}

	var inputPose commonv1.Pose
	var inputQuat map[string]interface{}

	t.Run("empty component reference success", func(t *testing.T) {
		svc.primarySensorName = ""
		inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}

		checkGetPositionModularizationV2Outputs(t, mockCartoFacade, inputPose, inputQuat, "", svc, originPos)
	})

	t.Run("origin pose success", func(t *testing.T) {
		svc.primarySensorName = "primarySensor1"
		inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}

		checkGetPositionModularizationV2Outputs(t, mockCartoFacade, inputPose, inputQuat, "primarySensor1", svc, originPos)
	})

	t.Run("non origin pose success", func(t *testing.T) {
		svc.primarySensorName = "primarySensor2"
		inputPose = commonv1.Pose{X: 5, Y: 5, Z: 5, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 1.0, "jmag": 0.0, "kmag": 0.0}

		checkGetPositionModularizationV2Outputs(t, mockCartoFacade, inputPose, inputQuat, "primarySensor2", svc, nonOriginPos)
	})

	t.Run("error case", func(t *testing.T) {
		svc.primarySensorName = "primarySensor3"
		mockCartoFacade.GetPositionFunc = func(
			ctx context.Context,
			timeout time.Duration,
		) (cartofacade.GetPosition, error) {
			return cartofacade.GetPosition{}, errors.New("testError")
		}

		inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}
		pose, componentReference, err := svc.GetPosition(context.Background())

		test.That(t, err, test.ShouldBeError, errors.New("testError"))
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentReference, test.ShouldEqual, "")
	})
}

//nolint:dupl
func TestGetPointCloudMapEndpoint(t *testing.T) {
	svc := &cartographerService{Named: resource.NewName(slam.API, "test").AsNamed()}
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

func setMockGetPointCloudFunc(
	mock *cartofacade.Mock,
	pc []byte,
) {
	mock.GetPointCloudMapFunc = func(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error) {
		return pc, nil
	}
}

func TestGetPointCloudMapEndpointModularizationV2Endpoint(t *testing.T) {
	svc := &cartographerService{Named: resource.NewName(slam.API, "test").AsNamed()}
	mockCartoFacade := &cartofacade.Mock{}
	pathToFileLargerThanChunkSize := "viam-cartographer/outputs/viam-office-02-22-3/pointcloud/pointcloud_0.pcd"
	pathToFileSmallerThanChunkSize := "viam-cartographer/outputs/viam-office-02-22-3/pointcloud/pointcloud_1.pcd"
	testApisThatReturnCallbackFuncs(
		t,
		svc,
		mockCartoFacade,
		pathToFileLargerThanChunkSize,
		pathToFileSmallerThanChunkSize,
		svc.GetPointCloudMap,
		setMockGetPointCloudFunc,
	)

	t.Run("cartofacade error", func(t *testing.T) {
		setMockGetPointCloudFunc(mockCartoFacade, []byte{})

		mockCartoFacade.GetPointCloudMapFunc = func(
			ctx context.Context,
			timeout time.Duration,
		) ([]byte, error) {
			return nil, errors.New("test")
		}

		callback, err := svc.GetPointCloudMap(context.Background())
		test.That(t, callback, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, errors.New("test"))
	})
}

//nolint:dupl
func TestGetInternalStateEndpoint(t *testing.T) {
	svc := &cartographerService{Named: resource.NewName(slam.API, "test").AsNamed()}
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

func setMockGetInternalStateFunc(
	mock *cartofacade.Mock,
	pc []byte,
) {
	mock.GetInternalStateFunc = func(
		ctx context.Context,
		timeout time.Duration,
	) ([]byte, error) {
		return pc, nil
	}
}

func testApisThatReturnCallbackFuncsSuccess(
	t *testing.T,
	path string,
	setMockFunc func(*cartofacade.Mock, []byte),
	mockCartoFacade *cartofacade.Mock,
	api func(context.Context) (func() ([]byte, error), error),
	fileIsLargerThanChunkSize bool,
) {
	bytes, err := os.ReadFile(artifact.MustPath(path))
	test.That(t, err, test.ShouldBeNil)

	if fileIsLargerThanChunkSize {
		test.That(t, len(bytes), test.ShouldBeGreaterThan, 1024*1024)
	} else {
		test.That(t, len(bytes), test.ShouldBeLessThan, 1024*1024)
	}

	setMockFunc(mockCartoFacade, bytes)

	getAndCheckCallbackFunc(t, api, false)
}

func getAndCheckCallbackFunc(
	t *testing.T,
	api func(context.Context) (func() ([]byte, error), error),
	emptyBytes bool,
) {
	callback, err := api(context.Background())
	test.That(t, err, test.ShouldBeNil)
	bytes, err := slam.HelperConcatenateChunksToFull(callback)
	test.That(t, err, test.ShouldBeNil)
	if emptyBytes {
		test.That(t, bytes, test.ShouldBeNil)
	} else {
		test.That(t, bytes, test.ShouldNotBeNil)
	}
}

func testApisThatReturnCallbackFuncs(
	t *testing.T,
	svc *cartographerService,
	mockCartoFacade *cartofacade.Mock,
	pathToFileLargerThanChunkSize string,
	pathToFileSmallerThanChunkSize string,
	api func(context.Context) (func() ([]byte, error), error),
	setMockFunc func(*cartofacade.Mock, []byte),
) {
	svc.cartofacade = mockCartoFacade
	svc.modularizationV2Enabled = true

	t.Run("returned value smaller than 1 mb limit - success", func(t *testing.T) {
		path := pathToFileLargerThanChunkSize
		testApisThatReturnCallbackFuncsSuccess(
			t,
			path,
			setMockFunc,
			mockCartoFacade,
			api,
			false,
		)
	})

	t.Run("returned value larger than 1 mb limit - success", func(t *testing.T) {
		path := pathToFileSmallerThanChunkSize
		testApisThatReturnCallbackFuncsSuccess(
			t,
			path,
			setMockFunc,
			mockCartoFacade,
			api,
			true,
		)
	})

	t.Run("no bytes success", func(t *testing.T) {
		setMockFunc(mockCartoFacade, []byte{})
		getAndCheckCallbackFunc(t, api, true)
	})
}

func TestGetInternalStateModularizationV2Endpoint(t *testing.T) {
	svc := &cartographerService{Named: resource.NewName(slam.API, "test").AsNamed()}
	mockCartoFacade := &cartofacade.Mock{}
	pathToFileLargerThanChunkSize := "viam-cartographer/outputs/viam-office-02-22-3/internal_state/internal_state_0.pbstream"
	pathToFileSmallerThanChunkSize := "viam-cartographer/outputs/viam-office-02-22-3/internal_state/internal_state_1.pbstream"
	testApisThatReturnCallbackFuncs(
		t,
		svc,
		mockCartoFacade,
		pathToFileLargerThanChunkSize,
		pathToFileSmallerThanChunkSize,
		svc.GetInternalState,
		setMockGetInternalStateFunc,
	)

	t.Run("cartofacade error", func(t *testing.T) {
		setMockGetInternalStateFunc(mockCartoFacade, []byte{})

		mockCartoFacade.GetInternalStateFunc = func(
			ctx context.Context,
			timeout time.Duration,
		) ([]byte, error) {
			return nil, errors.New("test")
		}

		callback, err := svc.GetInternalState(context.Background())
		test.That(t, callback, test.ShouldBeNil)
		test.That(t, err, test.ShouldBeError, errors.New("test"))
	})
}

func TestParseCartoAlgoConfig(t *testing.T) {
	logger := golog.NewTestLogger(t)

	t.Run("returns default when config params are empty", func(t *testing.T) {
		defaultCartoAlgoCfg := cartofacade.CartoAlgoConfig{
			OptimizeOnStart:      false,
			OptimizeEveryNNodes:  3,
			NumRangeData:         30,
			MissingDataRayLength: 25.0,
			MaxRange:             25.0,
			MinRange:             0.2,
			MaxSubmapsToKeep:     3,
			FreshSubmapsCount:    3,
			MinCoveredArea:       1.0,
			MinAddedSubmapsCount: 1,
			OccupiedSpaceWeight:  20.0,
			TranslationWeight:    10.0,
			RotationWeight:       1.0,
		}

		configParams := map[string]string{}
		cartoAlgoConfig, err := parseCartoAlgoConfig(configParams, logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, cartoAlgoConfig, test.ShouldResemble, defaultCartoAlgoCfg)
	})

	t.Run("returns overrides when config is non empty", func(t *testing.T) {
		configParams := map[string]string{
			"optimize_on_start":       "true",
			"optimize_every_n_nodes":  "1",
			"num_range_data":          "2",
			"missing_data_ray_length": "3.0",
			"max_range":               "4.0",
			"min_range":               "5.0",
			"max_submaps_to_keep":     "6",
			"fresh_submaps_count":     "7",
			"min_covered_area":        "8.0",
			"min_added_submaps_count": "9",
			"occupied_space_weight":   "10.0",
			"translation_weight":      "11.0",
			"rotation_weight":         "12.0",
		}

		overRidenCartoAlgoCfg := cartofacade.CartoAlgoConfig{
			OptimizeOnStart:      true,
			OptimizeEveryNNodes:  1,
			NumRangeData:         2,
			MissingDataRayLength: 3.0,
			MaxRange:             4.0,
			MinRange:             5.0,
			MaxSubmapsToKeep:     6,
			FreshSubmapsCount:    7,
			MinCoveredArea:       8.0,
			MinAddedSubmapsCount: 9,
			OccupiedSpaceWeight:  10.0,
			TranslationWeight:    11.0,
			RotationWeight:       12.0,
		}

		cartoAlgoConfig, err := parseCartoAlgoConfig(configParams, logger)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, cartoAlgoConfig, test.ShouldResemble, overRidenCartoAlgoCfg)
	})

	t.Run("returns error when unsupported param provided", func(t *testing.T) {
		configParams := map[string]string{
			"optimize_on_start": "true",
			"invalid_param":     "hihi",
		}

		_, err := parseCartoAlgoConfig(configParams, logger)
		test.That(t, err, test.ShouldBeNil)
	})

	t.Run("returns error when param type is invalid", func(t *testing.T) {
		configParams := map[string]string{
			"optimize_every_n_nodes": "hihi",
		}

		cartoAlgoConfig, err := parseCartoAlgoConfig(configParams, logger)
		test.That(t, err, test.ShouldBeError, errors.New("strconv.Atoi: parsing \"hihi\": invalid syntax"))
		test.That(t, cartoAlgoConfig, test.ShouldResemble, defaultCartoAlgoCfg)
	})
}
