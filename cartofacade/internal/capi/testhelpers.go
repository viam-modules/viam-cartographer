package capi

import (
	"os"
)

func GetTestConfig() (CartoConfig, string, error) {
	dir, err := os.MkdirTemp("", "slam-test")
	if err != nil {
		return CartoConfig{}, "", err
	}

	return CartoConfig{
		Sensors:            []string{"rplidar", "imu"},
		MapRateSecond:      5,
		DataDir:            dir,
		ComponentReference: "component",
		LidarConfig:        twoD,
	}, dir, nil
}

func GetBadTestConfig() CartoConfig {
	return CartoConfig{
		Sensors:     []string{"rplidar", "imu"},
		LidarConfig: twoD,
	}
}

func GetTestAlgoConfig() CartoAlgoConfig {
	return CartoAlgoConfig{
		optimizeOnStart:      false,
		optimizeEveryNNodes:  0,
		numRangeData:         0,
		missingDataRayLength: 0.0,
		maxRange:             0.0,
		minRange:             0.0,
		maxSubmapsToKeep:     0,
		freshSubmapsCount:    0,
		minCoveredArea:       0.0,
		minAddedSubmapsCount: 0,
		occupiedSpaceWeight:  0.0,
		translationWeight:    0.0,
		rotationWeight:       0.0,
	}
}
