// Package viamcartographer_test tests the functions that require injected components (such as robot and camera)
// in order to be run. It utilizes the internal package located in testhelper.go to access
// certain exported functions which we do not want to make available to the user.
package viamcartographer_test

import (
	"context"
	"net"
	"testing"

	"github.com/edaniels/golog"
	"go.viam.com/rdk/config"
	"go.viam.com/rdk/services/slam"
	slamConfig "go.viam.com/slam/config"
	"go.viam.com/test"
	"google.golang.org/grpc"

	viamcartographer "github.com/viamrobotics/viam-cartographer"
	"github.com/viamrobotics/viam-cartographer/internal/testhelper"
)

const (
	testExecutableName      = "true" // the program "true", not the boolean value
	validDataRateMsec       = 200
	dataBufferSize          = 4
	sensorTestMaxTimeoutSec = 1
	sensorTestIntervalSec   = 1
	dialMaxTimeoutSec       = 1
)

var (
	validMapRateMsec = 200
	_true            = true
	_false           = false
)

func setupTestGRPCServer(tb testing.TB) (*grpc.Server, int) {
	listener, err := net.Listen("tcp", ":0")
	test.That(tb, err, test.ShouldBeNil)
	grpcServer := grpc.NewServer()
	go grpcServer.Serve(listener)

	return grpcServer, listener.Addr().(*net.TCPAddr).Port
}

func createSLAMService(
	t *testing.T,
	attrCfg *slamConfig.AttrConfig,
	logger golog.Logger,
	bufferSLAMProcessLogs bool,
	executableName string,
) (slam.Service, error) {
	t.Helper()

	ctx := context.Background()
	cfgService := config.Service{Name: "test", Type: "slam", Model: viamcartographer.Model}
	cfgService.ConvertedAttributes = attrCfg

	deps := testhelper.SetupDeps(attrCfg.Sensors)

	sensorDeps, err := attrCfg.Validate("path")
	if err != nil {
		return nil, err
	}
	test.That(t, sensorDeps, test.ShouldResemble, attrCfg.Sensors)

	svc, err := viamcartographer.New(
		ctx,
		deps,
		cfgService,
		logger,
		bufferSLAMProcessLogs,
		executableName,
		sensorTestMaxTimeoutSec,
		sensorTestIntervalSec,
		dialMaxTimeoutSec,
	)
	if err != nil {
		test.That(t, svc, test.ShouldBeNil)
		return nil, err
	}

	test.That(t, svc, test.ShouldNotBeNil)
	return svc, nil
}
