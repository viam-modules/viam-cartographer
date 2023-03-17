// Package viamcartographer implements simultaneous localization and mapping
// This is an Experimental package
package viamcartographer

import (
	"bufio"
	"bytes"
	"context"
	"image"
	"image/jpeg"
	"io"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"github.com/mitchellh/mapstructure"
	"github.com/pkg/errors"
	"go.opencensus.io/trace"
	v1 "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/slam/v1"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/config"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/registry"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/services/slam/grpchelper"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/rdk/vision"
	slamConfig "go.viam.com/slam/config"
	"go.viam.com/slam/dataprocess"
	slamUtils "go.viam.com/slam/utils"
	goutils "go.viam.com/utils"
	"go.viam.com/utils/pexec"
	"go.viam.com/utils/protoutils"
)

var (
	// Model is the model name of cartographer.
	Model = resource.NewModel("viam", "slam", "cartographer")
	// BinaryLocation contains the name of the cartographer grpc server.
	BinaryLocation                = "carto_grpc_server"
	cameraValidationMaxTimeoutSec = 30 // reconfigurable for testing
	dialMaxTimeoutSec             = 30 // reconfigurable for testing
)

const (
	defaultDataRateMsec         = 200
	defaultMapRateSec           = 60
	cameraValidationIntervalSec = 1.
	parsePortMaxTimeoutSec      = 60
	opTimeoutErrorMessage       = "bad scan: OpTimeout"
	localhost0                  = "localhost:0"
)

// SubAlgo defines the cartographer specific sub-algorithms that we support.
type SubAlgo string

const dim2d SubAlgo = "2d"

// SetCameraValidationMaxTimeoutSecForTesting sets cameraValidationMaxTimeoutSec for testing.
func SetCameraValidationMaxTimeoutSecForTesting(val int) {
	cameraValidationMaxTimeoutSec = val
}

// SetDialMaxTimeoutSecForTesting sets dialMaxTimeoutSec for testing.
func SetDialMaxTimeoutSecForTesting(val int) {
	dialMaxTimeoutSec = val
}

func init() {
	registry.RegisterService(slam.Subtype, Model, registry.Service{Constructor: newCartographer})

	config.RegisterServiceAttributeMapConverter(slam.Subtype, Model,
		func(attributes config.AttributeMap) (interface{}, error) {
			var attrCfg slamConfig.AttrConfig
			decoder, err := mapstructure.NewDecoder(&mapstructure.DecoderConfig{TagName: "json", Result: &attrCfg})
			if err != nil {
				return nil, err
			}
			if err := decoder.Decode(attributes); err != nil {
				return nil, err
			}
			return &attrCfg, nil
		}, &slamConfig.AttrConfig{})
}

func newCartographer(ctx context.Context, deps registry.Dependencies, config config.Service, logger golog.Logger) (interface{}, error) {
	return New(ctx, deps, config, logger, false)
}

// runtimeServiceValidation ensures the service's data processing and saving is valid for the subAlgo and
// cameras given.
func runtimeServiceValidation(
	ctx context.Context,
	cams []camera.Camera,
	cartoSvc *cartographerService,
) error {
	if !cartoSvc.useLiveData {
		return nil
	}

	var err error
	var path string
	paths := make([]string, 0, 1)
	startTime := time.Now()

	for {
		path, err = cartoSvc.getAndSaveData(ctx, cams)
		paths = append(paths, path)

		if err == nil {
			break
		}

		// This takes about 5 seconds, so the timeout should be sufficient.
		if time.Since(startTime) >= time.Duration(cameraValidationMaxTimeoutSec)*time.Second {
			return errors.Wrap(err, "error getting data from sensor")
		}
		if !goutils.SelectContextOrWait(ctx, cameraValidationIntervalSec*time.Second) {
			return ctx.Err()
		}
	}

	for _, path := range paths {
		if err := os.RemoveAll(path); err != nil {
			return errors.Wrap(err, "error removing generated file during validation")
		}
	}

	return nil
}

// cartographerService is the structure of the slam service.
type cartographerService struct {
	generic.Unimplemented
	primarySensorName string
	subAlgo           SubAlgo
	slamProcess       pexec.ProcessManager
	clientAlgo        pb.SLAMServiceClient
	clientAlgoClose   func() error

	configParams        map[string]string
	dataDirectory       string
	deleteProcessedData bool
	useLiveData         bool

	port       string
	dataRateMs int
	mapRateSec int

	dev bool

	cancelFunc              func()
	logger                  golog.Logger
	activeBackgroundWorkers sync.WaitGroup

	bufferSLAMProcessLogs        bool
	slamProcessLogReader         io.ReadCloser
	slamProcessLogWriter         io.WriteCloser
	slamProcessBufferedLogReader bufio.Reader
}

// configureCameras will check the config to see if any cameras are desired and if so, grab the cameras from
// the robot. We assume there is at most one lidar camera.
func configureCameras(
	svcConfig *slamConfig.AttrConfig,
	deps registry.Dependencies,
	logger golog.Logger,
) (string, []camera.Camera, error) {
	if len(svcConfig.Sensors) > 0 {
		logger.Debug("Running in live mode")
		cams := make([]camera.Camera, 0, len(svcConfig.Sensors))
		// The first camera is expected to be LIDAR.
		primarySensorName := svcConfig.Sensors[0]
		cam, err := camera.FromDependencies(deps, primarySensorName)
		if err != nil {
			return "", nil, errors.Wrapf(err, "error getting camera %v for slam service", primarySensorName)
		}
		cams = append(cams, cam)
		return primarySensorName, cams, nil
	}
	return "", nil, nil
}

// Position forwards the request for positional data to the slam library's gRPC service. Once a response is received,
// it is unpacked into a PoseInFrame.
func (cartoSvc *cartographerService) Position(
	ctx context.Context,
	name string,
	extra map[string]interface{},
) (*referenceframe.PoseInFrame, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::Position")
	defer span.End()

	ext, err := protoutils.StructToStructPb(extra)
	if err != nil {
		return nil, err
	}

	var pInFrame *referenceframe.PoseInFrame
	var returnedExt map[string]interface{}

	// TODO: Once RSDK-1053 (https://viam.atlassian.net/browse/RSDK-1066) is complete the original code before extracting position
	// from GetPosition will be removed and the GetPositionNew -> GetPosition
	if cartoSvc.dev {
		cartoSvc.logger.Debug("IN DEV MODE (position request)")
		req := &pb.GetPositionNewRequest{Name: name}

		resp, err := cartoSvc.clientAlgo.GetPositionNew(ctx, req)
		if err != nil {
			return nil, errors.Wrap(err, "error getting SLAM position")
		}

		pInFrame = referenceframe.NewPoseInFrame(resp.GetComponentReference(), spatialmath.NewPoseFromProtobuf(resp.GetPose()))
		returnedExt = resp.Extra.AsMap()
	} else {
		//nolint:staticcheck
		req := &pb.GetPositionRequest{Name: name, Extra: ext}

		resp, err := cartoSvc.clientAlgo.GetPosition(ctx, req)
		if err != nil {
			return nil, errors.Wrap(err, "error getting SLAM position")
		}

		pInFrame = referenceframe.ProtobufToPoseInFrame(resp.Pose)
		returnedExt = resp.Extra.AsMap()
	}

	// TODO DATA-531: https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-531
	// Remove extraction and conversion of quaternion from the extra field in the response once the Rust
	// spatial math library is available and the desired math can be implemented on the orbSLAM side
	if val, ok := returnedExt["quat"]; ok {
		q := val.(map[string]interface{})

		valReal, ok1 := q["real"].(float64)
		valIMag, ok2 := q["imag"].(float64)
		valJMag, ok3 := q["jmag"].(float64)
		valKMag, ok4 := q["kmag"].(float64)

		if !ok1 || !ok2 || !ok3 || !ok4 {
			cartoSvc.logger.Debugf("quaternion given, but invalid format detected, %v, skipping quaternion transform", q)
			return pInFrame, nil
		}
		newPose := spatialmath.NewPose(pInFrame.Pose().Point(),
			&spatialmath.Quaternion{Real: valReal, Imag: valIMag, Jmag: valJMag, Kmag: valKMag})
		pInFrame = referenceframe.NewPoseInFrame(pInFrame.Parent(), newPose)
	}

	return pInFrame, nil
}

// GetPosition forwards the request for positional data to the slam library's gRPC service. Once a response is received,
// it is unpacked into a Pose and a component reference string.
func (cartoSvc *cartographerService) GetPosition(ctx context.Context, name string) (spatialmath.Pose, string, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::GetPosition")
	defer span.End()

	req := &pb.GetPositionNewRequest{Name: name}

	resp, err := cartoSvc.clientAlgo.GetPositionNew(ctx, req)
	if err != nil {
		return nil, "", errors.Wrap(err, "error getting SLAM position")
	}
	pose := spatialmath.NewPoseFromProtobuf(resp.GetPose())
	componentReference := resp.GetComponentReference()
	returnedExt := resp.Extra.AsMap()

	return slamUtils.CheckQuaternionFromClientAlgo(pose, componentReference, returnedExt)
}

// GetMap forwards the request for map data to the slam library's gRPC service. Once a response is received it is unpacked
// into a mimeType and either a vision.Object or image.Image.
func (cartoSvc *cartographerService) GetMap(
	ctx context.Context,
	name, mimeType string,
	cp *referenceframe.PoseInFrame,
	include bool,
	extra map[string]interface{},
) (
	string, image.Image, *vision.Object, error,
) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::GetMap")
	defer span.End()

	var cameraPosition *v1.Pose
	if cp != nil {
		cameraPosition = referenceframe.PoseInFrameToProtobuf(cp).Pose
	}

	ext, err := protoutils.StructToStructPb(extra)
	if err != nil {
		return "", nil, nil, err
	}

	var imData image.Image
	var vObj *vision.Object

	// TODO: Once RSDK-1053 (https://viam.atlassian.net/browse/RSDK-1066) is complete the original code that extracts
	// the map will be removed and GetMap will be changed to GetPointCloudMap
	if cartoSvc.dev {
		cartoSvc.logger.Debug("IN DEV MODE (map request)")

		//nolint:staticcheck
		reqPCMap := &pb.GetPointCloudMapRequest{
			Name: name,
		}

		if mimeType != rdkutils.MimeTypePCD {
			return "", nil, nil, errors.New("non-pcd return type is impossible in while in dev mode")
		}

		//nolint:staticcheck
		resp, err := cartoSvc.clientAlgo.GetPointCloudMap(ctx, reqPCMap)
		if err != nil {
			return "", imData, vObj, errors.Errorf("error getting SLAM map (%v) : %v", mimeType, err)
		}
		pointcloudData := resp.GetPointCloudPcd()
		if pointcloudData == nil {
			return "", nil, nil, errors.New("get map read pointcloud unavailable")
		}
		pc, err := pc.ReadPCD(bytes.NewReader(pointcloudData))
		if err != nil {
			return "", nil, nil, errors.Wrap(err, "get map read pointcloud failed")
		}

		vObj, err = vision.NewObject(pc)
		if err != nil {
			return "", nil, nil, errors.Wrap(err, "get map creating vision object failed")
		}

		mimeType = rdkutils.MimeTypePCD
	} else {
		//nolint:staticcheck
		req := &pb.GetMapRequest{
			Name:               name,
			MimeType:           mimeType,
			CameraPosition:     cameraPosition,
			IncludeRobotMarker: include,
			Extra:              ext,
		}

		//nolint:staticcheck
		resp, err := cartoSvc.clientAlgo.GetMap(ctx, req)
		if err != nil {
			return "", imData, vObj, errors.Errorf("error getting SLAM map (%v) : %v", mimeType, err)
		}

		switch mimeType {
		case rdkutils.MimeTypeJPEG:
			imData, err = jpeg.Decode(bytes.NewReader(resp.GetImage()))
			if err != nil {
				return "", nil, nil, errors.Wrap(err, "get map decode image failed")
			}
		case rdkutils.MimeTypePCD:
			pointcloudData := resp.GetPointCloud()
			if pointcloudData == nil {
				return "", nil, nil, errors.New("get map read pointcloud unavailable")
			}
			pc, err := pc.ReadPCD(bytes.NewReader(pointcloudData.PointCloud))
			if err != nil {
				return "", nil, nil, errors.Wrap(err, "get map read pointcloud failed")
			}

			vObj, err = vision.NewObject(pc)
			if err != nil {
				return "", nil, nil, errors.Wrap(err, "get map creating vision object failed")
			}
		}
		mimeType = resp.MimeType
	}

	return mimeType, imData, vObj, nil
}

// GetInternalState forwards the request for the SLAM algorithms's internal state. Once a response is received, it is returned
// to the user.
func (cartoSvc *cartographerService) GetInternalState(ctx context.Context, name string) ([]byte, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::GetInternalState")
	defer span.End()

	//nolint:staticcheck
	req := &pb.GetInternalStateRequest{Name: name}

	//nolint:staticcheck
	resp, err := cartoSvc.clientAlgo.GetInternalState(ctx, req)
	if err != nil {
		return nil, errors.Wrap(err, "error getting the internal state from the SLAM client")
	}

	internalState := resp.GetInternalState()
	return internalState, err
}

// GetPointCloudMapStream creates a request, calls the slam algorithms GetPointCloudMapStream endpoint and returns a callback
// function which will return the next chunk of the current pointcloud map.
func (cartoSvc *cartographerService) GetPointCloudMapStream(ctx context.Context, name string) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::GetPointCloudMapStream")
	defer span.End()

	return grpchelper.GetPointCloudMapStreamCallback(ctx, name, cartoSvc.clientAlgo)
}

// GetInternalStateStream creates a request, calls the slam algorithms GetInternalStateStream endpoint and returns a callback
// function which will return the next chunk of the current internal state of the slam algo.
func (cartoSvc *cartographerService) GetInternalStateStream(ctx context.Context, name string) (func() ([]byte, error), error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::GetInternalStateStream")
	defer span.End()

	return grpchelper.GetInternalStateStreamCallback(ctx, name, cartoSvc.clientAlgo)
}

// New returns a new slam service for the given robot.
func New(
	ctx context.Context,
	deps registry.Dependencies,
	config config.Service,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
) (slam.Service, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::slamService::New")
	defer span.End()

	svcConfig, ok := config.ConvertedAttributes.(*slamConfig.AttrConfig)
	if !ok {
		return nil, rdkutils.NewUnexpectedTypeError(svcConfig, config.ConvertedAttributes)
	}

	primarySensorName, cams, err := configureCameras(svcConfig, deps, logger)
	if err != nil {
		return nil, errors.Wrap(err, "configuring camera error")
	}

	subAlgo := SubAlgo(svcConfig.ConfigParams["mode"])
	if subAlgo != dim2d {
		return nil, errors.Errorf("%v does not have a subAlgo %v",
			string(config.Model.Name), svcConfig.ConfigParams["mode"])
	}

	err = slamConfig.SetupDirectories(svcConfig.DataDirectory, logger)
	if err != nil {
		return nil, err
	}

	port, dataRateMsec, mapRateSec, useLiveData, deleteProcessedData, err := slamConfig.GetOptionalParameters(
		svcConfig,
		localhost0,
		defaultDataRateMsec,
		defaultMapRateSec,
		logger,
	)
	if err != nil {
		return nil, err
	}
	cancelCtx, cancelFunc := context.WithCancel(ctx)

	// SLAM Service Object
	cartoSvc := &cartographerService{
		primarySensorName:     primarySensorName,
		subAlgo:               subAlgo,
		slamProcess:           pexec.NewProcessManager(logger),
		configParams:          svcConfig.ConfigParams,
		dataDirectory:         svcConfig.DataDirectory,
		useLiveData:           useLiveData,
		deleteProcessedData:   deleteProcessedData,
		port:                  port,
		dataRateMs:            dataRateMsec,
		mapRateSec:            mapRateSec,
		cancelFunc:            cancelFunc,
		logger:                logger,
		bufferSLAMProcessLogs: bufferSLAMProcessLogs,
		dev:                   svcConfig.Dev,
	}

	var success bool
	defer func() {
		if !success {
			if err := cartoSvc.Close(); err != nil {
				logger.Errorw("error closing out after error", "error", err)
			}
		}
	}()

	if err := runtimeServiceValidation(cancelCtx, cams, cartoSvc); err != nil {
		return nil, errors.Wrap(err, "runtime slam service error")
	}

	cartoSvc.StartDataProcess(cancelCtx, cams, nil)

	if err := cartoSvc.StartSLAMProcess(ctx); err != nil {
		return nil, errors.Wrap(err, "error with slam service slam process")
	}

	client, clientClose, err := slamConfig.SetupGRPCConnection(ctx, cartoSvc.port, dialMaxTimeoutSec, logger)
	if err != nil {
		return nil, errors.Wrap(err, "error with initial grpc client to slam algorithm")
	}
	cartoSvc.clientAlgo = client
	cartoSvc.clientAlgoClose = clientClose

	success = true
	return cartoSvc, nil
}

// Close out of all slam related processes.
func (cartoSvc *cartographerService) Close() error {
	defer func() {
		if cartoSvc.clientAlgoClose != nil {
			goutils.UncheckedErrorFunc(cartoSvc.clientAlgoClose)
		}
	}()
	cartoSvc.cancelFunc()
	if cartoSvc.bufferSLAMProcessLogs {
		if cartoSvc.slamProcessLogReader != nil {
			if err := cartoSvc.slamProcessLogReader.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log reader")
			}
		}
		if cartoSvc.slamProcessLogWriter != nil {
			if err := cartoSvc.slamProcessLogWriter.Close(); err != nil {
				return errors.Wrap(err, "error occurred during closeout of slam log writer")
			}
		}
	}
	if err := cartoSvc.StopSLAMProcess(); err != nil {
		return errors.Wrap(err, "error occurred during closeout of process")
	}
	cartoSvc.activeBackgroundWorkers.Wait()
	return nil
}

// StartDataProcess is the background control loop for sending data from camera to the data directory for processing.
func (cartoSvc *cartographerService) StartDataProcess(
	cancelCtx context.Context,
	cams []camera.Camera,
	c chan int,
) {
	if !cartoSvc.useLiveData {
		return
	}

	cartoSvc.activeBackgroundWorkers.Add(1)
	if err := cancelCtx.Err(); err != nil {
		if !errors.Is(err, context.Canceled) {
			cartoSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
		}
		cartoSvc.activeBackgroundWorkers.Done()
		return
	}
	goutils.PanicCapturingGo(func() {
		ticker := time.NewTicker(time.Millisecond * time.Duration(cartoSvc.dataRateMs))
		defer ticker.Stop()
		defer cartoSvc.activeBackgroundWorkers.Done()

		for {
			if err := cancelCtx.Err(); err != nil {
				if !errors.Is(err, context.Canceled) {
					cartoSvc.logger.Errorw("unexpected error in SLAM data process", "error", err)
				}
				return
			}

			select {
			case <-cancelCtx.Done():
				return
			case <-ticker.C:
				cartoSvc.activeBackgroundWorkers.Add(1)
				if err := cancelCtx.Err(); err != nil {
					if !errors.Is(err, context.Canceled) {
						cartoSvc.logger.Errorw("unexpected error in SLAM service", "error", err)
					}
					cartoSvc.activeBackgroundWorkers.Done()
					return
				}
				goutils.PanicCapturingGo(func() {
					defer cartoSvc.activeBackgroundWorkers.Done()
					if _, err := cartoSvc.getAndSaveData(cancelCtx, cams); err != nil {
						cartoSvc.logger.Warn(err)
					}
					if c != nil {
						c <- 1
					}
				})
			}
		}
	})
}

// GetSLAMProcessConfig returns the process config for the SLAM process.
func (cartoSvc *cartographerService) GetSLAMProcessConfig() pexec.ProcessConfig {
	var args []string

	args = append(args, "-sensors="+cartoSvc.primarySensorName)
	args = append(args, "-config_param="+slamUtils.DictToString(cartoSvc.configParams))
	args = append(args, "-data_rate_ms="+strconv.Itoa(cartoSvc.dataRateMs))
	args = append(args, "-map_rate_sec="+strconv.Itoa(cartoSvc.mapRateSec))
	args = append(args, "-data_dir="+cartoSvc.dataDirectory)
	args = append(args, "-delete_processed_data="+strconv.FormatBool(cartoSvc.deleteProcessedData))
	args = append(args, "-use_live_data="+strconv.FormatBool(cartoSvc.useLiveData))
	args = append(args, "-port="+cartoSvc.port)
	args = append(args, "--aix-auto-update")

	return pexec.ProcessConfig{
		ID:      "slam_cartographer",
		Name:    BinaryLocation,
		Args:    args,
		Log:     true,
		OneShot: false,
	}
}

func (cartoSvc *cartographerService) GetSLAMProcessBufferedLogReader() bufio.Reader {
	return cartoSvc.slamProcessBufferedLogReader
}

// startSLAMProcess starts up the SLAM library process by calling the executable binary and giving it the necessary arguments.
func (cartoSvc *cartographerService) StartSLAMProcess(ctx context.Context) error {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::slamService::StartSLAMProcess")
	defer span.End()

	processConfig := cartoSvc.GetSLAMProcessConfig()

	var logReader io.ReadCloser
	var logWriter io.WriteCloser
	var bufferedLogReader bufio.Reader
	if cartoSvc.port == localhost0 || cartoSvc.bufferSLAMProcessLogs {
		logReader, logWriter = io.Pipe()
		bufferedLogReader = *bufio.NewReader(logReader)
		processConfig.LogWriter = logWriter
	}

	_, err := cartoSvc.slamProcess.AddProcessFromConfig(ctx, processConfig)
	if err != nil {
		return errors.Wrap(err, "problem adding slam process")
	}

	cartoSvc.logger.Debug("starting slam process")

	if err = cartoSvc.slamProcess.Start(ctx); err != nil {
		return errors.Wrap(err, "problem starting slam process")
	}

	if cartoSvc.port == localhost0 {
		timeoutCtx, timeoutCancel := context.WithTimeout(ctx, parsePortMaxTimeoutSec*time.Second)
		defer timeoutCancel()

		if !cartoSvc.bufferSLAMProcessLogs {
			defer func(logger golog.Logger) {
				if err := logReader.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(cartoSvc.logger)
			defer func(logger golog.Logger) {
				if err := logWriter.Close(); err != nil {
					logger.Debugw("Closing logReader returned an error", "error", err)
				}
			}(cartoSvc.logger)
		}

		for {
			if err := timeoutCtx.Err(); err != nil {
				return errors.Wrapf(err, "error getting port from slam process")
			}

			line, err := bufferedLogReader.ReadString('\n')
			if err != nil {
				return errors.Wrapf(err, "error getting port from slam process")
			}
			portLogLinePrefix := "Server listening on "
			if strings.Contains(line, portLogLinePrefix) {
				linePieces := strings.Split(line, portLogLinePrefix)
				if len(linePieces) != 2 {
					return errors.Errorf("failed to parse port from slam process log line: %v", line)
				}
				cartoSvc.port = "localhost:" + strings.TrimRight(linePieces[1], "\n")
				break
			}
		}
	}

	if cartoSvc.bufferSLAMProcessLogs {
		cartoSvc.slamProcessLogReader = logReader
		cartoSvc.slamProcessLogWriter = logWriter
		cartoSvc.slamProcessBufferedLogReader = bufferedLogReader
	}

	return nil
}

// stopSLAMProcess uses the process manager to stop the created slam process from running.
func (cartoSvc *cartographerService) StopSLAMProcess() error {
	if err := cartoSvc.slamProcess.Stop(); err != nil {
		return errors.Wrap(err, "problem stopping slam process")
	}
	return nil
}

// getAndSaveData implements the data extraction for dense algos and saving to the directory path (data subfolder) specified in
// the config. It returns the full filepath for each file saved along with any error associated with the data creation or saving.
func (cartoSvc *cartographerService) getAndSaveData(ctx context.Context, cams []camera.Camera) (string, error) {
	ctx, span := trace.StartSpan(ctx, "viamcartographer::cartographerService::getAndSaveData")
	defer span.End()

	if len(cams) != 1 {
		return "", errors.Errorf("expected one lidar camera for cartographer, found %v", len(cams))
	}

	pointcloud, err := cams[0].NextPointCloud(ctx)
	if err != nil {
		if err.Error() == opTimeoutErrorMessage {
			cartoSvc.logger.Warnw("Skipping this scan due to error", "error", err)
			return "", nil
		}
		return "", err
	}

	fileType := ".pcd"
	filename := createTimestampFilenames(cartoSvc.dataDirectory, cartoSvc.primarySensorName, fileType)
	return filename, dataprocess.WritePCDToFile(pointcloud, filename)
}

// Creates a file for camera data with the specified sensor name and timestamp written into the filename.
// For RGBD cameras, two filenames are created with the same timestamp in different directories.
func createTimestampFilenames(dataDirectory, primarySensorName, fileType string) string {
	timeStamp := time.Now()
	dataDir := filepath.Join(dataDirectory, "data")
	return dataprocess.CreateTimestampFilename(dataDir, primarySensorName, fileType, timeStamp)
}
