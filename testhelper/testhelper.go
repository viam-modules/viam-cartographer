// Package testhelper provides test helpers which don't depend on viamcartographer
package testhelper

import (
	"bytes"
	"context"
	"fmt"
	"os"
	"path"
	"strconv"
	"sync/atomic"
	"testing"
	"time"

	"github.com/pkg/errors"
	"github.com/viamrobotics/gostream"
	"go.viam.com/components/camera/replaypcd"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/rdk/utils/contextutils"
	"go.viam.com/utils/artifact"

	s "github.com/viamrobotics/viam-cartographer/sensors"
)

const (
	// NumPointClouds is the number of pointclouds saved in artifact
	// for the cartographer integration tests.
	NumPointClouds = 15
	// TestTime can be used to test specific timestamps provided by a replay sensor.
	TestTime = "2006-01-02T15:04:05.9999Z"
	// BadTime can be used to represent something that should cause an error while parsing it as a time.
	BadTime = "NOT A TIME"
)

var mockLidarPath = artifact.MustPath("viam-cartographer/mock_lidar")

// IntegrationLidarReleasePointCloudChan is the lidar pointcloud release
// channel for the integration tests.
var IntegrationLidarReleasePointCloudChan = make(chan int, 1)

// SetupStubDeps returns stubbed dependencies based on the sensors
// the stubs fail tests if called.
func SetupStubDeps(sensors []string, t *testing.T) resource.Dependencies {
	deps := make(resource.Dependencies)

	for _, sensor := range sensors {
		switch sensor {
		case "stub_lidar":
			deps[camera.Named(sensor)] = getStubLidar(t)
		default:
			t.Errorf("SetupStubDeps calld with unhandled sensor sensors: %s, %v", sensor, sensors)
		}
	}
	return deps
}

func getStubLidar(t *testing.T) *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		t.Error("stub lidar NextPointCloud called")
		return nil, errors.New("invalid sensor")
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		t.Error("stub lidar Stream called")
		return nil, errors.New("invalid sensor")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		t.Error("stub lidar Projector called")
		return nil, transform.NewNoIntrinsicsError("")
	}
	return cam
}

// SetupDeps returns the dependencies based on the sensors passed as arguments.
func SetupDeps(sensors []string) resource.Dependencies {
	deps := make(resource.Dependencies)

	for _, sensor := range sensors {
		switch sensor {
		case "good_lidar":
			deps[camera.Named(sensor)] = getGoodLidar()
		case "warming_up_lidar":
			deps[camera.Named(sensor)] = getWarmingUpLidar()
		case "replay_sensor":
			deps[camera.Named(sensor)] = getReplaySensor(TestTime)
		case "invalid_replay_sensor":
			deps[camera.Named(sensor)] = getReplaySensor(BadTime)
		case "invalid_sensor":
			deps[camera.Named(sensor)] = getInvalidSensor()
		case "gibberish":
			return deps
		case "cartographer_int_lidar":
			deps[camera.Named(sensor)] = getIntegrationLidar()
		case "finished_replay_sensor":
			deps[camera.Named(sensor)] = getFinishedReplaySensor()
		default:
			continue
		}
	}
	return deps
}

func getWarmingUpLidar() *inject.Camera {
	cam := &inject.Camera{}
	counter := 0
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		counter++
		if counter == 1 {
			return nil, errors.Errorf("warming up %d", counter)
		}
		return pointcloud.New(), nil
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	return cam
}

func getGoodLidar() *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return pointcloud.New(), nil
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	return cam
}

func getReplaySensor(testTime string) *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		md := ctx.Value(contextutils.MetadataContextKey)
		if mdMap, ok := md.(map[string][]string); ok {
			mdMap[contextutils.TimeRequestedMetadataKey] = []string{testTime}
		}
		return pointcloud.New(), nil
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	return cam
}

func getInvalidSensor() *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return nil, errors.New("invalid sensor")
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("invalid sensor")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	return cam
}

func getIntegrationLidar() *inject.Camera {
	cam := &inject.Camera{}
	var index uint64
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		select {
		case <-IntegrationLidarReleasePointCloudChan:
			i := atomic.AddUint64(&index, 1) - 1
			if i >= NumPointClouds {
				return nil, errors.New("No more cartographer point clouds")
			}
			file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/" + strconv.FormatUint(i, 10) + ".pcd"))
			if err != nil {
				return nil, err
			}
			pointCloud, err := pointcloud.ReadPCD(file)
			if err != nil {
				return nil, err
			}
			return pointCloud, nil
		default:
			return nil, errors.Errorf("Lidar not ready to return point cloud %v", atomic.LoadUint64(&index))
		}
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	return cam
}

func getFinishedReplaySensor() *inject.Camera {
	cam := &inject.Camera{}
	cam.NextPointCloudFunc = func(ctx context.Context) (pointcloud.PointCloud, error) {
		return nil, replaypcd.ErrEndOfDataset
	}
	cam.StreamFunc = func(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
		return nil, errors.New("lidar not camera")
	}
	cam.ProjectorFunc = func(ctx context.Context) (transform.Projector, error) {
		return nil, transform.NewNoIntrinsicsError("")
	}
	cam.PropertiesFunc = func(ctx context.Context) (camera.Properties, error) {
		return camera.Properties{}, nil
	}
	return cam
}

func mockLidarReadingsValid() error {
	dirEntries, err := os.ReadDir(mockLidarPath)
	if err != nil {
		return err
	}

	var files []string
	for _, f := range dirEntries {
		if !f.IsDir() {
			files = append(files, f.Name())
		}
	}
	if len(files) < NumPointClouds {
		return errors.New("expected at least 15 lidar readings for integration test")
	}
	for i := 0; i < NumPointClouds; i++ {
		found := false
		expectedFile := fmt.Sprintf("%d.pcd", i)
		for _, file := range files {
			if file == expectedFile {
				found = true
				break
			}
		}

		if !found {
			return errors.Errorf("expected %s to exist for integration test", path.Join(mockLidarPath, expectedFile))
		}
	}
	return nil
}

// FullModIntegrationLidarTimedSensor returns a mock timed lidar sensor
// or an error if preconditions to build the mock are not met.
// It validates that all required mock lidar reading files are able to be found.
// When the mock is called, it returns the next mock lidar reading, with the
// ReadingTime incremented by the sensorReadingInterval.
// The Replay sensor field of the mock readings will match the replay paramenter.
// When the end of the mock lidar readings is reached, the done channel
// is written to once so the caller can detect all lidar readings have been emitted
// from the mock. This is intended to match the same "end of dataset" behavior of a
// replay sensor.
// It is imporntant to provide determanistic time information to cartographer to
// ensure test outputs of cartographer are determanistic.
func FullModIntegrationLidarTimedSensor(
	t *testing.T,
	sensor string,
	replay bool,
	sensorReadingInterval time.Duration,
	done chan struct{},
) (s.TimedSensor, error) {
	err := mockLidarReadingsValid()
	if err != nil {
		return nil, err
	}

	var i uint64
	closed := false

	ts := &s.TimedSensorMock{}
	readingTime := time.Date(2021, 8, 15, 14, 30, 45, 100, time.UTC)

	ts.TimedSensorReadingFunc = func(ctx context.Context) (s.TimedSensorReadingResponse, error) {
		readingTime = readingTime.Add(sensorReadingInterval)
		t.Logf("TimedSenorReading Mock i: %d, closed: %v, readingTime: %s\n", i, closed, readingTime.String())
		if i >= NumPointClouds {
			// communicate to the test that all lidar readings have been written
			if !closed {
				done <- struct{}{}
				closed = true
			}
			return s.TimedSensorReadingResponse{}, errors.New("end of dataset")
		}

		file, err := os.Open(artifact.MustPath("viam-cartographer/mock_lidar/" + strconv.FormatUint(i, 10) + ".pcd"))
		if err != nil {
			t.Error("TimedSenorReading Mock failed to open pcd file")
			return s.TimedSensorReadingResponse{}, err
		}
		readingPc, err := pointcloud.ReadPCD(file)
		if err != nil {
			t.Error("TimedSenorReading Mock failed to read pcd")
			return s.TimedSensorReadingResponse{}, err
		}

		buf := new(bytes.Buffer)
		err = pointcloud.ToPCD(readingPc, buf, pointcloud.PCDBinary)
		if err != nil {
			t.Error("TimedSenorReading Mock failed to parse pcd")
			return s.TimedSensorReadingResponse{}, err
		}

		i++
		return s.TimedSensorReadingResponse{Reading: buf.Bytes(), ReadingTime: readingTime, Replay: replay}, nil
	}

	return ts, nil
}
