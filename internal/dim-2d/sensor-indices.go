// Package dim2d implements cartographer's 2D sub algorithm
package dim2d

// In 'mode: "2d"', we expect exclusively a single 2d lidar.
var (
	// The 2D Lidar is expected to be located at the first
	// index in the provided `sensors` array in the slam
	// service configuration.
	lidar2DIndex = 0
)
