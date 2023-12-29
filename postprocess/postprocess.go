// package postproces contains functionality to postprocess
package postprocess

import (
	"bytes"
	"image/color"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

func UpdatePointCloud(
	data []byte,
	updatedData *[]byte,
	addedPoints []r3.Vector,
	removedPoints []r3.Vector,
) error {
	if len(addedPoints) != 0 {
		err := updatePointCloudWithAddedPoints(data, updatedData, addedPoints)
		if err != nil {
			return err
		}
	} else {
		*updatedData = append(*updatedData, data...)
	}

	if len(removedPoints) != 0 {
		err := updatePointCloudWithRemovedPoints(updatedData, removedPoints)
		if err != nil {
			return err
		}
	}

	return nil
}

func updatePointCloudWithAddedPoints(data []byte, updatedData *[]byte, points []r3.Vector) error {
	const FULL_CONFIDENCE = 100

	reader := bytes.NewReader(data)
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
