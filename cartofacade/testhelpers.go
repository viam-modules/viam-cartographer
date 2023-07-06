package cartofacade

import (
	"os"
)

// GetTestConfig gets a sample config for testing purposes.
func GetTestConfig(sensor string) (CartoConfig, string, error) {
	dir, err := os.MkdirTemp("", "slam-test")
	if err != nil {
		return CartoConfig{}, "", err
	}

	return CartoConfig{
		Sensors:            []string{sensor, "imu"},
		MapRateSecond:      5,
		DataDir:            dir,
		ComponentReference: "component",
		LidarConfig:        twoD,
	}, dir, nil
}

// GetBadTestConfig gets a sample config for testing purposes that will cause a failure.
func GetBadTestConfig() CartoConfig {
	return CartoConfig{
		Sensors:     []string{"rplidar", "imu"},
		LidarConfig: twoD,
	}
}

// GetTestAlgoConfig gets a sample algo config for testing purposes.
func GetTestAlgoConfig() CartoAlgoConfig {
	return CartoAlgoConfig{
		optimizeOnStart:      false,
		optimizeEveryNNodes:  3,
		numRangeData:         100,
		missingDataRayLength: 25.0,
		maxRange:             25.0,
		minRange:             0.2,
		maxSubmapsToKeep:     3,
		freshSubmapsCount:    3,
		minCoveredArea:       1.0,
		minAddedSubmapsCount: 1,
		occupiedSpaceWeight:  20.0,
		translationWeight:    10.0,
		rotationWeight:       1.0,
	}
}
