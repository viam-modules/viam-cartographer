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
		LidarConfig:        TwoD,
	}, dir, nil
}

// GetBadTestConfig gets a sample config for testing purposes that will cause a failure.
func GetBadTestConfig() CartoConfig {
	return CartoConfig{
		Sensors:     []string{"rplidar", "imu"},
		LidarConfig: TwoD,
	}
}

// GetTestAlgoConfig gets a sample algo config for testing purposes.
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
