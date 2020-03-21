include "map_builder.lua"
include "trajectory_builder.lua"

---Edited on 12.05.2019
-- Pointcloud Quanergy config for cloud processing

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
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

--MAP_BUILDER.use_trajectory_builder_3d = false -- default false
MAP_BUILDER.use_trajectory_builder_2d = true -- default false

TRAJECTORY_BUILDER_2D.use_imu_data = true --Do NOT use IMU for generation of trajectory
TRAJECTORY_BUILDER_2D.min_range = 1 -- Min range of the LIDAR, default 0 
TRAJECTORY_BUILDER_2D.max_range = 40. -- Max range of the LIDAR, default 30
--TRAJECTORY_BUILDER_2D.submaps.num_range_data  =  90-- Size of submaps Default 90
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
--TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 160 -- This kinda works likes sampling. Setting 0 = no sampling. Default 1
--TRAJECTORY_BUILDER_2D.laser_min_range = 0.01
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false



-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e1
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3
-- TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 300

-- POSE_GRAPH.optimization_problem.huber_scale = 1e2
-- POSE_GRAPH.optimize_every_n_nodes = 90 -- Optimization takes time. Faster the better, depends on computing power, default 90
--POSE_GRAPH.constraint_builder.min_score = 0.60 -- minimum score for local loop closure, default 0.55
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65 --min score for global loop closure, default 0,6
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.003 -- which ratio of scans/nodes the constraint builder tries to use for loop closing, default 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 5 -- default 10
-- MAP_BUILDER.pose_graph.constraint_builder.max_constraint_distance = 200

return options
