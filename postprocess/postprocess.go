// package postproces contains functionality to postprocess pointcloud maps
package postprocess

import (
	"bytes"
	"errors"
	"image/color"
	"reflect"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

type Instruction int

const (
	AddPointsInstruction    Instruction = iota
	RemovePointsInstruction             = iota
)

const (
	// expected strings for DoCommand
	ToggleCommand = "postprocess_toggle"
	AddCommand    = "postprocess_add"
	RemoveCommand = "postprocess_remove"
	UndoCommand   = "postprocess_undo"
)

var (
	// ErrBadPostprocessingPointsFormat denotes that the points have not been properly formatted
	ErrBadPostprocessingPointsFormat = errors.New("could not parse provided points")
)

type Task struct {
	Instruction Instruction
	Points      []r3.Vector
}

func ParseDoCommand(
	unstructuredPoints interface{},
	instruction Instruction,
) (Task, error) {
	if reflect.TypeOf(unstructuredPoints).Kind() != reflect.Slice {
		return Task{}, ErrBadPostprocessingPointsFormat
	}

	task := Task{}
	slice := reflect.ValueOf(unstructuredPoints)

	for i := 0; i < slice.Len(); i++ {
		val := slice.Index(i).Elem()
		if reflect.TypeOf(val).Kind() != reflect.Struct {
			return Task{}, ErrBadPostprocessingPointsFormat
		}

		x := val.MapIndex(reflect.ValueOf("X"))
		y := val.MapIndex(reflect.ValueOf("Y"))
		task.Points = append(task.Points, r3.Vector{X: x.Elem().Float(), Y: y.Elem().Float()})
	}

	task.Instruction = instruction
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
		case AddPointsInstruction:
			err := updatePointCloudWithAddedPoints(updatedData, task.Points)
			if err != nil {
				return err
			}
		case RemovePointsInstruction:
			err := updatePointCloudWithRemovedPoints(updatedData, task.Points)
			if err != nil {
				return err
			}
		}
	}
	return nil
}

func updatePointCloudWithAddedPoints(updatedData *[]byte, points []r3.Vector) error {
	const FULL_CONFIDENCE = 100

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
		pc.Set(point, pointcloud.NewColoredData(color.NRGBA{B: FULL_CONFIDENCE}))
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
