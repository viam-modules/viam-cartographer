include "trajectory_builder.lua"
include "map_builder.lua"

-- !!!!! NOTE: DO NOT CHANGE THIS FILE !!!!! --

-- ALWAYS TRY TO TUNE FOR A SENSOR SET/ROBOT SETUP,
-- NOT A PARTICULAR ENVIRONMENT --

-- ===== Local SLAM Options ======
-- no reason to change these:
TRAJECTORY_BUILDER.trajectory_builder_2d.use_imu_data = false
TRAJECTORY_BUILDER.trajectory_builder_2d.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER.trajectory_builder_2d.min_range = 0.2
TRAJECTORY_BUILDER.trajectory_builder_2d.max_range = 25.
TRAJECTORY_BUILDER.trajectory_builder_2d.missing_data_ray_length = 25 -- DO NOT CHANGE

-- tuneable:
TRAJECTORY_BUILDER.trajectory_builder_2d.submaps.num_range_data = 30
TRAJECTORY_BUILDER.trajectory_builder_2d.ceres_scan_matcher.occupied_space_weight = 20.
TRAJECTORY_BUILDER.trajectory_builder_2d.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER.trajectory_builder_2d.ceres_scan_matcher.rotation_weight = 1.

-- ===== Global SLAM Options ======
-- no reason to change these:
MAP_BUILDER.use_trajectory_builder_2d = true
-- tuneable:
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 3

-- ===== Return Options ======
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
}

return options




