package cartofacade

// GetTestConfig gets a sample config for testing purposes.
func GetTestConfig(cameraName, movementSensorName, filename string, enableMapping bool) CartoConfig {
	return CartoConfig{
		Camera:         cameraName,
		MovementSensor: movementSensorName,
		LidarConfig:    TwoD,
		EnableMapping:  enableMapping,
		ExistingMap:    filename,
	}
}

// GetBadTestConfig gets a sample config for testing purposes that will cause a failure.
func GetBadTestConfig() CartoConfig {
	return CartoConfig{
		LidarConfig: TwoD,
	}
}

// GetTestAlgoConfig gets a sample algo config for testing purposes.
func GetTestAlgoConfig(useImuData bool) CartoAlgoConfig {
	return CartoAlgoConfig{
		OptimizeOnStart:      false,
		OptimizeEveryNNodes:  3,
		NumRangeData:         100,
		MissingDataRayLength: 25.0,
		MaxRange:             25.0,
		MinRange:             0.2,
		UseIMUData:           useImuData,
		MaxSubmapsToKeep:     3,
		FreshSubmapsCount:    3,
		MinCoveredArea:       1.0,
		MinAddedSubmapsCount: 1,
		OccupiedSpaceWeight:  20.0,
		TranslationWeight:    10.0,
		RotationWeight:       1.0,
	}
}
