// Package testhelper implements a slam service definition with additional exported functions for
// the purpose of testing. It does also contain helper variables and functions that are
// used across all tests in the viam-cartographer repo.
package testhelper

import (
	"bufio"
	"context"
	"fmt"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/google/uuid"
	"github.com/pkg/errors"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/test"
	"go.viam.com/utils/artifact"
	"go.viam.com/utils/pexec"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	vcConfig "github.com/viamrobotics/viam-cartographer/config"
	"github.com/viamrobotics/viam-cartographer/dataprocess"
	s "github.com/viamrobotics/viam-cartographer/sensors"
	"github.com/viamrobotics/viam-cartographer/sensors/lidar"
	externaltesthelper "github.com/viamrobotics/viam-cartographer/testhelper"
)

const (
	// SensorValidationMaxTimeoutSecForTest is used in the ValidateGetAndSaveData
	// function to ensure that the sensor in the GetAndSaveData function
	// returns data within an acceptable time.
	SensorValidationMaxTimeoutSecForTest = 1
	// SensorValidationIntervalSecForTest is used in the ValidateGetAndSaveData
	// function for the while loop that attempts to grab data from the
	// sensor that is used in the GetAndSaveData function.
	SensorValidationIntervalSecForTest = 1
	testDialMaxTimeoutSec              = 1
)

// Service in the internal package includes additional exported functions relating to the data and
// slam processes in the slam service. These functions are not exported to the user. This resolves
// a circular import caused by the inject package.
type Service interface {
	StartDataProcess(cancelCtx context.Context, lidar lidar.Lidar, c chan int)
	StartSLAMProcess(ctx context.Context) error
	StopSLAMProcess() error
	Close(ctx context.Context) error
	GetSLAMProcessConfig() pexec.ProcessConfig
	GetSLAMProcessBufferedLogReader() bufio.Reader
}

// ClearDirectory deletes the contents in the path directory
// without deleting path itself.
func ClearDirectory(t *testing.T, path string) {
	t.Helper()

	err := ResetFolder(path)
	test.That(t, err, test.ShouldBeNil)
}

// CreateFullModSLAMServiceIntegration creates a slam service for testing.
func CreateFullModSLAMServiceIntegration(
	t *testing.T,
	cfg *vcConfig.Config,
	timedSensor s.TimedSensor,
	logger golog.Logger,
) (slam.Service, error) {
	ctx := context.Background()
	cfgService := resource.Config{Name: getTestName(), API: slam.API, Model: viamcartographer.Model}
	cfgService.ConvertedAttributes = cfg

	sensorDeps, err := cfg.Validate("path")
	if err != nil {
		return nil, err
	}
	test.That(t, sensorDeps, test.ShouldResemble, cfg.Sensors)
	deps := externaltesthelper.SetupStubDeps(cfg.Sensors, t)

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		true,
		"true",
		SensorValidationMaxTimeoutSecForTest,
		SensorValidationIntervalSecForTest,
		testDialMaxTimeoutSec,
		5*time.Second,
		timedSensor,
	)
	if err != nil {
		test.That(t, svc, test.ShouldBeNil)
		return nil, err
	}

	test.That(t, svc, test.ShouldNotBeNil)

	return svc, nil
}

// CreateSLAMService creates a slam service for testing.
func CreateSLAMService(
	t *testing.T,
	cfg *vcConfig.Config,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
	executableName string,
) (slam.Service, error) {
	t.Helper()

	ctx := context.Background()
	cfgService := resource.Config{Name: getTestName(), API: slam.API, Model: viamcartographer.Model}
	cfgService.ConvertedAttributes = cfg

	deps := externaltesthelper.SetupDeps(cfg.Sensors)

	sensorDeps, err := cfg.Validate("path")
	if err != nil {
		return nil, err
	}
	test.That(t, sensorDeps, test.ShouldResemble, cfg.Sensors)

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		bufferSLAMProcessLogs,
		executableName,
		SensorValidationMaxTimeoutSecForTest,
		SensorValidationIntervalSecForTest,
		testDialMaxTimeoutSec,
		5*time.Second,
		nil,
	)
	if err != nil {
		test.That(t, svc, test.ShouldBeNil)
		return nil, err
	}

	test.That(t, svc, test.ShouldNotBeNil)

	return svc, nil
}

// CheckDeleteProcessedData compares the number of files found in a specified data
// directory with the previous number found and uses the useLiveData and
// deleteProcessedData values to evaluate this comparison. It returns the number of files
// currently in the data directory for the specified config. Future invocations should pass in this
// value. This function should be passed 0 as a default prev argument in order to get the
// number of files currently in the directory.
func CheckDeleteProcessedData(
	t *testing.T,
	subAlgo viamcartographer.SubAlgo,
	dir string,
	prev int,
	deleteProcessedData,
	useLiveData bool,
) int {
	switch subAlgo {
	case viamcartographer.Dim2d:
		numFiles, err := CheckDataDirForExpectedFiles(t, dir+"/data", prev, deleteProcessedData, useLiveData)
		test.That(t, err, test.ShouldBeNil)
		return numFiles
	default:
		return 0
	}
}

const (
	dataBufferSize = 4
)

// CreateTempFolderArchitecture creates a new random temporary
// directory with the config, data, and map subdirectories needed
// to run the SLAM libraries.
func CreateTempFolderArchitecture(logger golog.Logger) (string, error) {
	tmpDir, err := os.MkdirTemp("", "*")
	if err != nil {
		return "", err
	}
	if err := vcConfig.SetupDirectories(tmpDir, logger); err != nil {
		return "", err
	}
	return tmpDir, nil
}

// ResetFolder removes all content in path and creates a new directory
// in its place.
func ResetFolder(path string) error {
	dirInfo, err := os.Stat(path)
	if err != nil {
		return err
	}
	if !dirInfo.IsDir() {
		return errors.Errorf("the path passed ResetFolder does not point to a folder: %v", path)
	}
	if err = os.RemoveAll(path); err != nil {
		return err
	}
	return os.Mkdir(path, dirInfo.Mode())
}

// CheckDataDirForExpectedFiles ensures that the provided data directory contains the correct amount of files
// based on the config parameters deleteProcessedData and useLiveData.
func CheckDataDirForExpectedFiles(t *testing.T, dir string, prev int, deleteProcessedData, useLiveData bool) (int, error) {
	files, err := os.ReadDir(dir)
	test.That(t, err, test.ShouldBeNil)

	if prev == 0 {
		return len(files), nil
	}
	if deleteProcessedData && useLiveData {
		test.That(t, prev, test.ShouldBeLessThanOrEqualTo, dataBufferSize+1)
	}
	if !deleteProcessedData && useLiveData {
		test.That(t, prev, test.ShouldBeLessThan, len(files))
	}
	if deleteProcessedData && !useLiveData {
		return 0, errors.New("the delete_processed_data value cannot be true when running SLAM in offline mode")
	}
	if !deleteProcessedData && !useLiveData {
		test.That(t, prev, test.ShouldEqual, len(files))
	}
	return len(files), nil
}

// InitTestCL initializes the carto library & returns a function to terminate it.
func InitTestCL(t *testing.T, logger golog.Logger) func() {
	t.Helper()
	err := viamcartographer.InitCartoLib(logger)
	test.That(t, err, test.ShouldBeNil)
	return func() {
		err = viamcartographer.TerminateCartoLib()
		test.That(t, err, test.ShouldBeNil)
	}
}

// InitInternalState creates the internal state directory witghin a temp directory
// with an internal state pbstream file & returns the data directory & a function
// to delete the data directory.
func InitInternalState(t *testing.T) (string, func()) {
	dataDirectory, err := os.MkdirTemp("", "*")
	test.That(t, err, test.ShouldBeNil)

	internalStateDir := filepath.Join(dataDirectory, "internal_state")
	err = os.Mkdir(internalStateDir, os.ModePerm)
	test.That(t, err, test.ShouldBeNil)

	file := "viam-cartographer/outputs/viam-office-02-22-3/internal_state/internal_state_0.pbstream"
	internalState, err := os.ReadFile(artifact.MustPath(file))
	test.That(t, err, test.ShouldBeNil)

	timestamp := time.Date(2006, 1, 2, 15, 4, 5, 999900000, time.UTC)
	filename := dataprocess.CreateTimestampFilename(dataDirectory+"/internal_state", "internal_state", ".pbstream", timestamp)
	err = os.WriteFile(filename, internalState, os.ModePerm)
	test.That(t, err, test.ShouldBeNil)

	return dataDirectory, func() {
		err := os.RemoveAll(dataDirectory)
		test.That(t, err, test.ShouldBeNil)
	}
}

func getTestName() string {
	id := uuid.New()
	return fmt.Sprintf("test-%s", id)
}
