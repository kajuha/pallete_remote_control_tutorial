-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link", -- imu_link, If you are using gazebo, use 'base_footprint' (libgazebo_ros_imu's bug)
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1, -- modified
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.1,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.1,
  landmarks_sampling_ratio = 1.,
}

-- MAP BUILDER
MAP_BUILDER.use_trajectory_builder_2d = true -- false
MAP_BUILDER.use_trajectory_builder_3d = false -- false
MAP_BUILDER.num_background_threads = 4 -- 4
-- MAP_BUILDER.pose_graph = POSE_GRAPH -- POSE_GRAPH

-- TRAJECTORY BUILDER
TRAJECTORY_BUILDER.pure_localization = true -- false
TRAJECTORY_BUILDER.pure_localization_trimmer ={
  max_submaps_to_keep = 3,
}

-- TRAJECTORY BUILDER 2D
TRAJECTORY_BUILDER_2D.use_imu_data = true                                    -- true
TRAJECTORY_BUILDER_2D.min_range = 0.1                                         -- 0.
TRAJECTORY_BUILDER_2D.max_range = 30.                                         -- 30.
TRAJECTORY_BUILDER_2D.min_z = -0.8                                            -- -0.8
TRAJECTORY_BUILDER_2D.max_z = 2.                                              -- 2.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.                            -- 5.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2                          -- 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025                                -- 0.025

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5                  -- 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200              -- 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.                   -- 50.

TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9     -- 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100 -- 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.      -- 50.

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false             -- false

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1             -- 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)  -- math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1   -- 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1      -- 1e-1

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.                           -- 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.                             -- 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.                                -- 40.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false  -- false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20         -- 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1                 -- 1

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.             -- 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2         -- 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) -- math.rad(1.)

TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10. -- 10.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90                                                               -- 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"                                    -- "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05                                                 -- 0.05
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D"     -- "PROBABILITY_GRID_INSERTER_2D"
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true -- true
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55   -- 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49  -- 0.49              

-- POSE GRAPH
POSE_GRAPH.optimize_every_n_nodes = 90 -- 90

POSE_GRAPH.constraint_builder.sampling_ratio = 0.3                                                -- 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.                                       -- 15.
POSE_GRAPH.constraint_builder.min_score = 0.55                                                    -- 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6                                 -- 0.6
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4                             -- 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5                                  -- 1e5
POSE_GRAPH.constraint_builder.log_matches = true                                                  -- true

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.             -- 7.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.) -- math.rad(30.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7            -- 7

POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.  -- 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.     -- 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.         -- 1.

POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true -- true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10       -- 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1               -- 1

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 8            -- 8
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 3             -- 3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.77           -- 0.77
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.55       -- 0.55
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5.          -- 5.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.           -- 1.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(15.) -- math.rad(15.)

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 5.                         -- 5.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 30.                        -- 30.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 10.                             -- 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 1.                                 -- 1.
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = false                            -- false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.use_nonmonotonic_steps = false  -- false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 10         -- 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 1                 -- 1

POSE_GRAPH.matcher_translation_weight = 5e2 -- 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3  -- 1.6e3

POSE_GRAPH.optimization_problem.huber_scale = 1e1                                   -- 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3                           -- 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 3e5                               -- 3e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5            -- 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5               -- 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5                   -- 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5                      -- 1e5
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1           -- 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e1              -- 1e1
POSE_GRAPH.optimization_problem.log_solver_summary = false                          -- false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false -- false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50        -- 50
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 7                -- 7

POSE_GRAPH.max_num_final_iterations = 200                 -- 200
POSE_GRAPH.global_sampling_ratio = 0.003                 -- 0.003
POSE_GRAPH.log_residual_histograms = true                 -- true
POSE_GRAPH.global_constraint_search_after_n_seconds = 5. -- 10.

return options
