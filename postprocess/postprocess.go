// Package postprocess contains functionality to postprocess pointcloud maps
package postprocess

import (
	"bytes"
	"errors"
	"image/color"
	"math"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

// Instruction describes the action of the postprocess step.
type Instruction int

const (
	// Add is the instruction for adding points.
	Add Instruction = iota
	// Remove is the instruction for removing points.
	Remove = iota
)

var logger = golog.NewDebugLogger("postprocess")

const (
	fullConfidence = 100
	removalRadius  = 100 // mm
	xKey           = "X"
	yKey           = "Y"

	// ToggleCommand can be used to turn postprocessing on and off.
	ToggleCommand = "postprocess_toggle"
	// AddCommand can be used to add points to the pointcloud  map.
	AddCommand = "postprocess_add"
	// RemoveCommand can be used to remove points from the pointcloud  map.
	RemoveCommand = "postprocess_remove"
	// UndoCommand can be used to undo last postprocessing step.
	UndoCommand = "postprocess_undo"
)

var (
	errPointsNotASlice = errors.New("could not parse provided points as a slice")
	errPointNotAMap    = errors.New("could not parse provided point as a map")
	errXNotProvided    = errors.New("could X not provided")
	errXNotFloat64     = errors.New("could not parse provided X as a float64")
	errYNotProvided    = errors.New("could X not provided")
	errYNotFloat64     = errors.New("could not parse provided X as a float64")
	errRemovingPoints  = errors.New("unexpected number of points after removal")
	errNilUpdatedData  = errors.New("cannot provide nil updated data")
)

// Task can be used to construct a postprocessing step.
type Task struct {
	Instruction Instruction
	Points      []r3.Vector
}

// ParseDoCommand parses postprocessing DoCommands into Tasks.
func ParseDoCommand(
	unstructuredPoints interface{},
	instruction Instruction,
) (Task, error) {
	logger.Info("DEBUGLOGS: parsingDoCommand")
	pointSlice, ok := unstructuredPoints.([]interface{})
	if !ok {
		return Task{}, errPointsNotASlice
	}

	task := Task{Instruction: instruction}
	for _, point := range pointSlice {
		pointMap, ok := point.(map[string]interface{})
		if !ok {
			return Task{}, errPointNotAMap
		}

		x, ok := pointMap[xKey]
		if !ok {
			return Task{}, errXNotProvided
		}

		xFloat, ok := x.(float64)
		if !ok {
			return Task{}, errXNotFloat64
		}

		y, ok := pointMap[yKey]
		if !ok {
			return Task{}, errYNotProvided
		}

		yFloat, ok := y.(float64)
		if !ok {
			return Task{}, errXNotFloat64
		}

		task.Points = append(task.Points, r3.Vector{X: xFloat, Y: yFloat})
	}
	return task, nil
}

/*
UpdatePointCloud iterated through a list of tasks and adds or removes points from data
and writes the updated pointcloud to updatedData.
*/
func UpdatePointCloud(
	data []byte,
	updatedData *[]byte,
	tasks []Task,
) error {
	logger.Info("DEBUGLOGS: UpdatePointCloud")
	if updatedData == nil {
		return errNilUpdatedData
	}

	*updatedData = append(*updatedData, data...)

	// iterate through tasks and add or remove points
	for _, task := range tasks {
		switch task.Instruction {
		case Add:
			logger.Info("DEBUGLOGS: got an add task")
			err := updatePointCloudWithAddedPoints(updatedData, task.Points)
			if err != nil {
				return err
			}
		case Remove:
			logger.Info("DEBUGLOGS: got an remove task")
			err := updatePointCloudWithRemovedPoints(updatedData, task.Points)
			if err != nil {
				return err
			}
		}
	}
	return nil
}

func updatePointCloudWithAddedPoints(updatedData *[]byte, points []r3.Vector) error {
	logger.Info("DEBUGLOGS: updatePointCloudWithAddedPoints")
	if updatedData == nil {
		return errNilUpdatedData
	}

	reader := bytes.NewReader(*updatedData)
	logger.Info("DEBUGLOGS: bytes.NewReader(*updatedData)")
	pc, err := pointcloud.ReadPCD(reader)
	logger.Info("DEBUGLOGS: read pcd")
	if err != nil {
		return err
	}

	for _, point := range points {
		logger.Infof("DEBUGLOGS: on point %v", point)
		/*
			Viam expects pointcloud data with fields "x y z" or "x y z rgb", and for
			this to be specified in the pointcloud header in the FIELDS entry. If color
			data is included in the pointcloud, Viam's services assume that the color
			value encodes a confidence score for that data point. Viam expects the
			confidence score to be encoded in the blue parameter of the RGB value, on a
			scale from 1-100.
		*/
		err := pc.Set(point, pointcloud.NewColoredData(color.NRGBA{B: fullConfidence, R: math.MaxUint8}))
		if err != nil {
			return err
		}
	}
	logger.Info("DEBUGLOGS: finished iterating through points")

	var buf bytes.Buffer
	err = pointcloud.ToPCD(pc, &buf, pointcloud.PCDBinary)
	if err != nil {
		return err
	}
	logger.Info("DEBUGLOGS: toPCD")

	// Initialize updatedData with new points
	*updatedData = make([]byte, buf.Len())
	logger.Info("DEBUGLOGS: new byte slice")
	updatedReader := bytes.NewReader(buf.Bytes())
	logger.Info("DEBUGLOGS: read byte buffer")
	_, err = updatedReader.Read(*updatedData)
	logger.Info("DEBUGLOGS: read from reader")
	if err != nil {
		return err
	}

	return nil
}

func updatePointCloudWithRemovedPoints(updatedData *[]byte, points []r3.Vector) error {
	logger.Info("DEBUGLOGS: updatePointCloudWithRemovedPoints")
	if updatedData == nil {
		return errNilUpdatedData
	}

	reader := bytes.NewReader(*updatedData)
	logger.Info("DEBUGLOGS: NewReader")
	pc, err := pointcloud.ReadPCD(reader)
	logger.Info("DEBUGLOGS: ReadPCD")
	if err != nil {
		return err
	}

	logger.Info("DEBUGLOGS: NewWithPrealloc")
	updatedPC := pointcloud.NewWithPrealloc(pc.Size() - len(points))
	pointsVisited := 0

	filterRemovedPoints := func(p r3.Vector, d pointcloud.Data) bool {
		logger.Info("DEBUGLOGS: filterRemovedPoints")
		pointsVisited++
		// Always return true so iteration continues

		for _, point := range points {
			// remove all points within the removalRadius from the removed points
			if point.Distance(p) <= removalRadius {
				return true
			}
		}

		err := updatedPC.Set(p, d)
		// end early if point cannot be set
		return err == nil
	}

	pc.Iterate(0, 0, filterRemovedPoints)
	logger.Info("DEBUGLOGS: pc.Iterate")

	// confirm iterate did not have to end early
	if pc.Size() != pointsVisited {
		/*
			Note: this condition will occur if some error occurred while copying valid points
			and will be how we can tell that this error occurred: err := updatedPC.Set(p, d)
		*/
		return errRemovingPoints
	}
	logger.Info("DEBUGLOGS: pc.Iterate")

	buf := bytes.Buffer{}
	err = pointcloud.ToPCD(updatedPC, &buf, pointcloud.PCDBinary)
	logger.Info("DEBUGLOGS: ToPCD")
	if err != nil {
		return err
	}

	// Overwrite updatedData with new points
	*updatedData = make([]byte, buf.Len())
	logger.Info("DEBUGLOGS: new byte slice")
	updatedReader := bytes.NewReader(buf.Bytes())
	logger.Info("DEBUGLOGS: read byte buffer")
	_, err = updatedReader.Read(*updatedData)
	logger.Info("DEBUGLOGS: read from reader")
	if err != nil {
		return err
	}

	return nil
}
