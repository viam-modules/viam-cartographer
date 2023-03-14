// Package testhelper implements a slam service definition with additional exported functions for
// the purpose of testing
package testhelper

import (
	"bufio"
	"context"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/utils/pexec"
)

// Service in the internal package includes additional exported functions relating to the data and
// slam processes in the slam service. These functions are not exported to the user. This resolves
// a circular import caused by the inject package.
type Service interface {
	StartDataProcess(cancelCtx context.Context, cam []camera.Camera, c chan int)
	StartSLAMProcess(ctx context.Context) error
	StopSLAMProcess() error
	Close() error
	GetSLAMProcessConfig() pexec.ProcessConfig
	GetSLAMProcessBufferedLogReader() bufio.Reader
}
