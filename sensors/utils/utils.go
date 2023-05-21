// Package utils contains helper functions for the sensor implementations.
package utils

import (
	"github.com/pkg/errors"
)

// GetName returns the name of the sensor based on its index in the sensor array.
func GetName(sensors []string, index int) (string, error) {
	if index < 0 || index >= len(sensors) {
		return "", errors.New("index out of bounds")
	}
	return sensors[index], nil
}
