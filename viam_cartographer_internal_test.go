package viamcartographer

import (
	"context"
	"math"
	"os"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	commonv1 "go.viam.com/api/common/v1"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"

	"github.com/viamrobotics/viam-cartographer/cartofacade"
)

func makeQuaternionFromGenericMap(quat map[string]interface{}) spatialmath.Orientation {
	return &spatialmath.Quaternion{
		Real: quat["real"].(float64),
		Imag: quat["imag"].(float64),
		Jmag: quat["jmag"].(float64),
		Kmag: quat["kmag"].(float64),
	}
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

func checkGetPositionOutputs(
	t *testing.T,
	mockCartoFacade *cartofacade.Mock,
	inputPose *commonv1.Pose,
	inputQuat map[string]interface{},
	inputComponentRef string,
	svc *CartographerService,
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

func TestGetPositionEndpoint(t *testing.T) {
	svc := &CartographerService{Named: resource.NewName(slam.API, "test").AsNamed()}
	mockCartoFacade := &cartofacade.Mock{}
	svc.cartofacade = mockCartoFacade

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
		svc.lidar.name = ""
		inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}

		checkGetPositionOutputs(t, mockCartoFacade, &inputPose, inputQuat, "", svc, originPos)
	})

	t.Run("origin pose success", func(t *testing.T) {
		svc.lidar.name = "primarySensor1"
		inputPose = commonv1.Pose{X: 0, Y: 0, Z: 0, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 0.0, "jmag": 0.0, "kmag": 0.0}

		checkGetPositionOutputs(t, mockCartoFacade, &inputPose, inputQuat, "primarySensor1", svc, originPos)
	})

	t.Run("non origin pose success", func(t *testing.T) {
		svc.lidar.name = "primarySensor2"
		inputPose = commonv1.Pose{X: 5, Y: 5, Z: 5, OX: 0, OY: 0, OZ: 1, Theta: 0}
		inputQuat = map[string]interface{}{"real": 1.0, "imag": 1.0, "jmag": 0.0, "kmag": 0.0}

		checkGetPositionOutputs(t, mockCartoFacade, &inputPose, inputQuat, "primarySensor2", svc, nonOriginPos)
	})

	t.Run("error case", func(t *testing.T) {
		svc.lidar.name = "primarySensor3"
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
	getAndCheckCallbackFunc(t, api, false, bytes)
}

func getAndCheckCallbackFunc(
	t *testing.T,
	api func(context.Context) (func() ([]byte, error), error),
	emptyBytes bool,
	inputBytes []byte,
) {
	callback, err := api(context.Background())
	test.That(t, err, test.ShouldBeNil)
	bytes, err := slam.HelperConcatenateChunksToFull(callback)
	test.That(t, err, test.ShouldBeNil)

	if emptyBytes {
		test.That(t, bytes, test.ShouldBeNil)
	} else {
		test.That(t, bytes, test.ShouldNotBeNil)
		test.That(t, bytes, test.ShouldResemble, inputBytes)
	}
}

func testApisThatReturnCallbackFuncs(
	t *testing.T,
	svc *CartographerService,
	mockCartoFacade *cartofacade.Mock,
	pathToFileLargerThanChunkSize string,
	pathToFileSmallerThanChunkSize string,
	api func(context.Context) (func() ([]byte, error), error),
	setMockFunc func(*cartofacade.Mock, []byte),
) {
	svc.cartofacade = mockCartoFacade

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
		getAndCheckCallbackFunc(t, api, true, []byte{})
	})
}

func TestGetPointCloudMapEndpoint(t *testing.T) {
	svc := &CartographerService{Named: resource.NewName(slam.API, "test").AsNamed()}
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

func TestGetInternalStateEndpoint(t *testing.T) {
	svc := &CartographerService{Named: resource.NewName(slam.API, "test").AsNamed()}
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

	t.Run("returns overrides when config is non empty for cloud story configs", func(t *testing.T) {
		configParams := map[string]string{
			"optimize_on_start":               "true",
			"optimize_every_n_nodes":          "1",
			"num_range_data":                  "2",
			"missing_data_ray_length_meters":  "3.0",
			"max_range_meters":                "4.0",
			"min_range_meters":                "5.0",
			"max_submaps_to_keep":             "6",
			"fresh_submaps_count":             "7",
			"min_covered_area_meters_squared": "8.0",
			"min_added_submaps_count":         "9",
			"occupied_space_weight":           "10.0",
			"translation_weight":              "11.0",
			"rotation_weight":                 "12.0",
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

func TestBuiltinQuaternion(t *testing.T) {
	poseSucc := spatialmath.NewPose(r3.Vector{X: 1, Y: 2, Z: 3}, &spatialmath.OrientationVector{Theta: math.Pi / 2, OX: 0, OY: 0, OZ: -1})
	componentRefSucc := "cam"
	t.Run("test successful quaternion from internal server", func(t *testing.T) {
		returnedExtSucc := map[string]interface{}{
			"quat": map[string]interface{}{
				"real": poseSucc.Orientation().Quaternion().Real,
				"imag": poseSucc.Orientation().Quaternion().Imag,
				"jmag": poseSucc.Orientation().Quaternion().Jmag,
				"kmag": poseSucc.Orientation().Quaternion().Kmag,
			},
		}

		pose, componentRef, err := CheckQuaternionFromClientAlgo(poseSucc, componentRefSucc, returnedExtSucc)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, spatialmath.PoseAlmostEqual(poseSucc, pose), test.ShouldBeTrue)
		test.That(t, componentRef, test.ShouldEqual, componentRefSucc)
	})

	t.Run("test failure due to quaternion not being given", func(t *testing.T) {
		returnedExtFail := map[string]interface{}{
			"badquat": map[string]interface{}{
				"real": poseSucc.Orientation().Quaternion().Real,
				"imag": poseSucc.Orientation().Quaternion().Imag,
				"jmag": poseSucc.Orientation().Quaternion().Jmag,
				"kmag": poseSucc.Orientation().Quaternion().Kmag,
			},
		}

		pose, componentRef, err := CheckQuaternionFromClientAlgo(poseSucc, componentRefSucc, returnedExtFail)
		test.That(t, err.Error(), test.ShouldContainSubstring, "error getting SLAM position: quaternion not given")
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentRef, test.ShouldBeEmpty)
	})

	t.Run("test failure due to invalid quaternion format", func(t *testing.T) {
		returnedExtFail := map[string]interface{}{
			"quat": map[string]interface{}{
				"realbad": poseSucc.Orientation().Quaternion().Real,
				"imagbad": poseSucc.Orientation().Quaternion().Imag,
				"jmagbad": poseSucc.Orientation().Quaternion().Jmag,
				"kmagbad": poseSucc.Orientation().Quaternion().Kmag,
			},
		}

		pose, componentRef, err := CheckQuaternionFromClientAlgo(poseSucc, componentRefSucc, returnedExtFail)
		test.That(t, err.Error(), test.ShouldContainSubstring, "error getting SLAM position: quaternion given, but invalid format detected")
		test.That(t, pose, test.ShouldBeNil)
		test.That(t, componentRef, test.ShouldBeEmpty)
	})
}
