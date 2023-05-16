// Package testhelper provides helper functions for testing implementations of slam libraries.
package testhelper

import (
	"os"
	"testing"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	"go.viam.com/test"

	"github.com/viamrobotics/viam-cartographer/config"
)

const (
	dataBufferSize = 4
)

// CreateTempFolderArchitecture creates a new random temporary
// directory with the config, data, and map subdirectories needed
// to run the SLAM libraries.
func CreateTempFolderArchitecture(logger golog.Logger) (string, error) {
	tmpDir, err := os.MkdirTemp("", "*")
	if err != nil {
		return "nil", err
	}
	if err := config.SetupDirectories(tmpDir, logger); err != nil {
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
