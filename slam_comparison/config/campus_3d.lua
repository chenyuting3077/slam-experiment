-- Cartographer 3D config for LIO-SAM Campus dataset (single VLP-16 + IMU)
-- Based on cartographer_ros backpack_3d.lua, adapted for a single rotating
-- 360-degree LiDAR instead of two actuated line lasers.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- VLP-16: single full 360-degree rotation per PointCloud2 message at 10Hz,
-- so unlike the actuated backpack lasers we don't need to accumulate many
-- partial scans to get a usable 3D point cloud.
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 100.

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 90  -- back to default; 320 was copied from
                                        -- backpack_3d.lua and just delayed
                                        -- pose-graph correction unnecessarily
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10

-- Default max_constraint_distance=15 means Cartographer never even attempts
-- a constraint between submaps whose centers are more than 15m apart --
-- exactly the same class of bug we found and fixed in LIO-SAM
-- (historyKeyframeSearchRadius 15->60). Zero loop closures were found on a
-- 1437m outdoor loop with real accumulated drift; the loop-return point is
-- almost certainly further than 15m from the corresponding old submap.
POSE_GRAPH.constraint_builder.max_constraint_distance = 60.
-- Also revert our previous (mistaken, overly conservative) overrides of
-- sampling_ratio/min_score back toward the shipped defaults (0.3/0.55),
-- erring slightly more generous since we don't need to keep up with a
-- live sensor here.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
