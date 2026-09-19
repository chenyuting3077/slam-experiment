-- Cartographer 3D config for the Zenodo Mid360 outdoor_hard_01 dataset.
-- Based on campus_3d.lua (VLP-16) -- same structure, adjusted for the
-- Mid360's non-repetitive scan pattern and shorter usable range.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- Point cloud and IMU both arrive with frame_id "livox_frame" already
  -- (Mid360's integrated IMU needs no separate extrinsic), so using it
  -- directly as the tracking frame avoids needing a static transform.
  tracking_frame = "livox_frame",
  published_frame = "livox_frame",
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

-- Mid360 publishes one aggregated PointCloud2 per ~0.13s (~7.5Hz), already
-- built up from many internal sub-scans by livox_ros_driver2 -- similar
-- enough to VLP-16's one-rotation-per-message to not need accumulating
-- several messages before matching.
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.3    -- Mid360 blind zone is ~0.1-0.2m
TRAJECTORY_BUILDER_3D.max_range = 60.    -- Mid360 spec range ~40-70m depending on reflectivity

-- The pose extrapolator's local odom->livox_frame estimate reproducibly
-- diverges (quadratically growing to tens of millions of meters within
-- ~1 minute of real-time playback). Tried imu_gravity_time_constant at
-- 1s (submap insertion stalled entirely -- worse), the 10s default
-- (diverges), and 30s (diverges even FASTER, ~-62M m by 95s in, worse
-- than default) -- tuning this parameter in either direction doesn't
-- fix it, which rules it out as the lever and points to something more
-- fundamental (most likely the pose extrapolator's IMU-integration-based
-- initial guess failing to track this "hard" sequence's aggressive
-- motion) rather than a simple gravity-convergence-speed issue. Left at
-- the shipped default; documented as an unresolved limitation.

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10

-- Same loop-closure search-radius fix already found necessary for the
-- Campus dataset -- start with the widened value straight away instead of
-- re-discovering the same bug.
POSE_GRAPH.constraint_builder.max_constraint_distance = 60.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
