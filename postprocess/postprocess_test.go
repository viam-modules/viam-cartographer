package postprocess

import (
	"bytes"
	"errors"
	"fmt"
	"image/color"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/test"
)

type TestCase struct {
	msg string
	cmd interface{}
	err error
}

func TestParseDoCommand(t *testing.T) {
	for _, tc := range []TestCase{
		{
			msg: "errors if unstructuredPoints is not a slice",
			cmd: "hello",
			err: ErrPointsNotASlice,
		},
		{
			msg: "errors if unstructuredPoints is not a slice of maps",
			cmd: []interface{}{1},
			err: ErrPointNotAMap,
		},
		{
			msg: "errors if unstructuredPoints contains a point where X is not float64",
			cmd: []interface{}{map[string]interface{}{"Y": float64(2)}},
			err: ErrXNotProvided,
		},
		{
			msg: "errors if unstructuredPoints contains a point where X is not float64",
			cmd: []interface{}{map[string]interface{}{"X": 1, "Y": float64(2)}},
			err: ErrXNotFloat64,
		},
		{
			msg: "errors if unstructuredPoints contains a point where Y is not provided",
			cmd: []interface{}{map[string]interface{}{"X": float64(1)}},
			err: ErrYNotProvided,
		},
		{
			msg: "errors if unstructuredPoints contains a point where Y is not float64",
			cmd: []interface{}{map[string]interface{}{"X": float64(1), "Y": 2}},
			err: ErrYNotFloat64,
		},
	} {
		t.Run(fmt.Sprintf("%s for Add task", tc.msg), func(t *testing.T) {
			task, err := ParseDoCommand(tc.cmd, Add)
			test.That(t, err, test.ShouldBeError, tc.err)
			test.That(t, task, test.ShouldResemble, Task{})
		})

		t.Run(fmt.Sprintf("%s for Remove task", tc.msg), func(t *testing.T) {
			task, err := ParseDoCommand(tc.cmd, Remove)
			test.That(t, err, test.ShouldBeError, tc.err)
			test.That(t, task, test.ShouldResemble, Task{})
		})
	}

	t.Run("succeeds if unstructuredPoints is a slice of maps with float64 values", func(t *testing.T) {
		expectedPoint := r3.Vector{X: 1, Y: 2}
		task, err := ParseDoCommand([]interface{}{map[string]interface{}{"X": float64(1), "Y": float64(2)}}, Add)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, task, test.ShouldResemble, Task{Instruction: Add, Points: []r3.Vector{expectedPoint}})
	})
}

func TestUpdatePointCloudWithAddedPoints(t *testing.T) {
	t.Run("errors if byte slice cannot be converted to PCD", func(t *testing.T) {
		originalPointsBytes := []byte("hello")
		err := updatePointCloudWithAddedPoints(&originalPointsBytes, []r3.Vector{{X: 2, Y: 2}, {X: 3, Y: 3}})
		test.That(t, err, test.ShouldBeError, errors.New("error reading header line 0: EOF"))
	})

	t.Run("successfully returns point cloud with postprocessed points", func(t *testing.T) {
		originalPoints := []r3.Vector{{X: 0, Y: 0}, {X: 1, Y: 1}}
		var originalPointsBytes []byte
		err := vecSliceToBytes(originalPoints, &originalPointsBytes)
		test.That(t, err, test.ShouldBeNil)

		postprocessedPoints := []r3.Vector{{X: 0, Y: 0}, {X: 1, Y: 1}, {X: 2, Y: 2}, {X: 3, Y: 3}}
		var postprocessedPointsBytes []byte
		err = vecSliceToBytes(postprocessedPoints, &postprocessedPointsBytes)
		test.That(t, err, test.ShouldBeNil)

		// update original byte slice with new points
		err = updatePointCloudWithAddedPoints(&originalPointsBytes, []r3.Vector{{X: 2, Y: 2}, {X: 3, Y: 3}})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, postprocessedPointsBytes, test.ShouldResemble, originalPointsBytes)
	})
}

func TestUpdatePointCloudWithRemovedPoints(t *testing.T) {
	t.Run("errors if byte slice cannot be converted to PCD", func(t *testing.T) {
		originalPointsBytes := []byte("hello")
		err := updatePointCloudWithRemovedPoints(&originalPointsBytes, []r3.Vector{{X: 2, Y: 2}, {X: 3, Y: 3}})
		test.That(t, err, test.ShouldBeError, errors.New("error reading header line 0: EOF"))
	})

	t.Run("successfully returns point cloud with postprocessed points", func(t *testing.T) {
		originalPoints := []r3.Vector{{X: 0, Y: 0}, {X: 1000, Y: 1000}, {X: 2000, Y: 2000}, {X: 2020, Y: 2020}, {X: 3000, Y: 3000}}
		var originalPointsBytes []byte
		err := vecSliceToBytes(originalPoints, &originalPointsBytes)
		test.That(t, err, test.ShouldBeNil)

		postprocessedPoints := []r3.Vector{{X: 0, Y: 0}, {X: 1000, Y: 1000}}
		var postprocessedPointsBytes []byte
		err = vecSliceToBytes(postprocessedPoints, &postprocessedPointsBytes)
		test.That(t, err, test.ShouldBeNil)

		// update original byte slice with new points
		err = updatePointCloudWithRemovedPoints(&originalPointsBytes, []r3.Vector{{X: 2000, Y: 2000}, {X: 3000, Y: 3000}})
		test.That(t, err, test.ShouldBeNil)
		test.That(t, postprocessedPointsBytes, test.ShouldResemble, originalPointsBytes)
	})
}

func TestUpdatePointCloud(t *testing.T) {
	originalPoints := []r3.Vector{{X: 0, Y: 0}, {X: 1000, Y: 1000}, {X: 2000, Y: 2000}, {X: 3000, Y: 3000}}
	var originalPointsBytes []byte
	err := vecSliceToBytes(originalPoints, &originalPointsBytes)
	test.That(t, err, test.ShouldBeNil)

	postprocessedPoints := []r3.Vector{
		{X: 0, Y: 0},
		{X: 1000, Y: 1000},
		{X: 3000, Y: 3000},
		{X: 5000, Y: 5000},
	}
	var postprocessedPointsBytes []byte
	err = vecSliceToBytes(postprocessedPoints, &postprocessedPointsBytes)
	test.That(t, err, test.ShouldBeNil)

	tasks := []Task{
		{
			Instruction: Add,
			Points:      []r3.Vector{{X: 4000, Y: 4000}, {X: 5000, Y: 5000}},
		},
		{
			Instruction: Remove,
			Points:      []r3.Vector{{X: 2000, Y: 2000}, {X: 4000, Y: 4000}},
		},
	}
	var updatedData []byte
	err = UpdatePointCloud(originalPointsBytes, &updatedData, tasks)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, updatedData, test.ShouldResemble, postprocessedPointsBytes)
}

func vecSliceToBytes(points []r3.Vector, outputData *[]byte) error {
	pc := pointcloud.NewWithPrealloc(len(points))
	for _, p := range points {
		pc.Set(p, pointcloud.NewColoredData(color.NRGBA{B: fullConfidence}))
	}

	buf := bytes.Buffer{}
	err := pointcloud.ToPCD(pc, &buf, pointcloud.PCDBinary)
	if err != nil {
		return err
	}

	// Initialize updatedData with new points
	*outputData = make([]byte, buf.Len())
	updatedReader := bytes.NewReader(buf.Bytes())
	updatedReader.Read(*outputData)
	return nil
}
