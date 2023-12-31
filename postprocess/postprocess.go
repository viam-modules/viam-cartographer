// package postproces contains functionality to postprocess pointcloud maps
package postprocess

import (
	"bytes"
	"errors"
	"image/color"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

type Instruction int

const (
	Add    Instruction = iota
	Remove             = iota
)

const (
	fullConfidence = 100

	// expected strings for DoCommand
	ToggleCommand = "postprocess_toggle"
	AddCommand    = "postprocess_add"
	RemoveCommand = "postprocess_remove"
	UndoCommand   = "postprocess_undo"
	xKey          = "X"
	yKey          = "Y"
)

var (
	// ErrPointsNotASlice denotes that the points have not been properly formatted as a slice
	ErrPointsNotASlice = errors.New("could not parse provided points as a slice")

	// ErrPointNotAMap denotes that a point has not been properly formatted as a map
	ErrPointNotAMap = errors.New("could not parse provided point as a map")

	// ErrXNotProvided denotes that an X value is not a float64
	ErrXNotProvided = errors.New("could X not provided")

	// ErrXNotFloat64 denotes that an X value is not a float64
	ErrXNotFloat64 = errors.New("could not parse provided X as a float64")

	// ErrYNotProvided denotes that an X value is not a float64
	ErrYNotProvided = errors.New("could X not provided")

	// ErrXNotFloat64 denotes that an X value is not a float64
	ErrYNotFloat64 = errors.New("could not parse provided X as a float64")
)

type Task struct {
	Instruction Instruction
	Points      []r3.Vector
}

func ParseDoCommand(
	unstructuredPoints interface{},
	instruction Instruction,
) (Task, error) {
	pointSlice, ok := unstructuredPoints.([]interface{})
	if !ok {
		return Task{}, ErrPointsNotASlice
	}

	task := Task{}
	for _, point := range pointSlice {
		pointMap, ok := point.(map[string]interface{})
		if !ok {
			return Task{}, ErrPointNotAMap
		}

		x, ok := pointMap[xKey]
		if !ok {
			return Task{}, ErrXNotProvided
		}

		xFloat, ok := x.(float64)
		if !ok {
			return Task{}, ErrXNotFloat64
		}

		y, ok := pointMap[yKey]
		if !ok {
			return Task{}, ErrYNotProvided
		}

		yFloat, ok := y.(float64)
		if !ok {
			return Task{}, ErrXNotFloat64
		}

		task.Points = append(task.Points, r3.Vector{X: xFloat, Y: yFloat})
	}
	return task, nil
}

func UpdatePointCloud(
	data []byte,
	updatedData *[]byte,
	tasks []Task,
) error {
	*updatedData = append(*updatedData, data...)

	// iterate through tasks and add or remove points
	for _, task := range tasks {
		switch task.Instruction {
		case Add:
			err := updatePointCloudWithAddedPoints(updatedData, task.Points)
			if err != nil {
				return err
			}
		case Remove:
			err := updatePointCloudWithRemovedPoints(updatedData, task.Points)
			if err != nil {
				return err
			}
		}
	}
	return nil
}

func updatePointCloudWithAddedPoints(updatedData *[]byte, points []r3.Vector) error {
	reader := bytes.NewReader(*updatedData)
	pc, err := pointcloud.ReadPCD(reader)
	if err != nil {
		return err
	}

	for _, point := range points {
		/*
			Viam expects pointcloud data with fields "x y z" or "x y z rgb", and for
			this to be specified in the pointcloud header in the FIELDS entry. If color
			data is included in the pointcloud, Viam's services assume that the color
			value encodes a confidence score for that data point. Viam expects the
			confidence score to be encoded in the blue parameter of the RGB value, on a
			scale from 1-100.
		*/
		pc.Set(point, pointcloud.NewColoredData(color.NRGBA{B: fullConfidence}))
	}

	var buf bytes.Buffer
	err = pointcloud.ToPCD(pc, &buf, pointcloud.PCDBinary)
	if err != nil {
		return err
	}

	// Initalize updatedData with new points
	*updatedData = make([]byte, buf.Len())
	updatedReader := bytes.NewReader(buf.Bytes())
	updatedReader.Read(*updatedData)

	return nil
}

func updatePointCloudWithRemovedPoints(updatedData *[]byte, points []r3.Vector) error {
	reader := bytes.NewReader(*updatedData)
	pc, err := pointcloud.ReadPCD(reader)
	if err != nil {
		return err
	}

	updatedPC := pointcloud.NewWithPrealloc(pc.Size() - len(points))

	filterRemovedPoints := func(p r3.Vector, d pointcloud.Data) bool {
		// Always return true so iteration continues

		for _, point := range points {
			if point == p {
				return true
			}
		}

		updatedPC.Set(p, d)
		return true
	}

	pc.Iterate(0, 0, filterRemovedPoints)

	buf := bytes.Buffer{}
	err = pointcloud.ToPCD(updatedPC, &buf, pointcloud.PCDBinary)
	if err != nil {
		return err
	}

	// Overwrite updatedData with new points
	*updatedData = make([]byte, buf.Len())
	updatedReader := bytes.NewReader(buf.Bytes())
	updatedReader.Read(*updatedData)

	return nil
}
