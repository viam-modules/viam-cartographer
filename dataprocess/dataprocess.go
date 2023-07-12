// Package dataprocess manages code related to the data-saving process.
package dataprocess

import (
	"bufio"
	"bytes"
	"encoding/json"
	"os"
	"path/filepath"
	"time"

	"github.com/golang/geo/r3"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

const (
	// SlamTimeFormat is the timestamp format used in the dataprocess.
	SlamTimeFormat = "2006-01-02T15:04:05.0000Z"
)

// CreateTimestampFilename creates an absolute filename with a primary sensor name and timestamp written
// into the filename.
func CreateTimestampFilename(dataDirectory, primarySensorName, fileType string, timeStamp time.Time) string {
	return filepath.Join(dataDirectory, primarySensorName+"_data_"+timeStamp.UTC().Format(SlamTimeFormat)+fileType)
}

// WritePCDToFile encodes the pointcloud and then saves it to the passed filename.
func WritePCDToFile(pointcloud pc.PointCloud, filename string) error {
	buf := new(bytes.Buffer)
	if err := pc.ToPCD(pointcloud, buf, 1); err != nil {
		return err
	}
	return WriteBytesToFile(buf.Bytes(), filename)
}

// WriteJSONToFile encodes the imu data and then saves it to the passed filename.
func WriteJSONToFile(linearAcceleration r3.Vector, angularVelocity spatialmath.AngularVelocity, filename string) error {
	buf := new(bytes.Buffer)

	jsonLinAcc, err := json.Marshal(linearAcceleration)
	if err != nil {
		return err
	}
	jsonAngVel, err := json.Marshal(angularVelocity)
	if err != nil {
		return err
	}
	buf.Write(jsonLinAcc)
	buf.Write(jsonAngVel)
	return WriteBytesToFile(buf.Bytes(), filename)
}

// WriteBytesToFile writes the passed bytes to the passed filename.
func WriteBytesToFile(bytes []byte, filename string) error {
	//nolint:gosec
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	w := bufio.NewWriter(f)
	if _, err := w.Write(bytes); err != nil {
		return err
	}
	if err := w.Flush(); err != nil {
		return err
	}
	return f.Close()
}
