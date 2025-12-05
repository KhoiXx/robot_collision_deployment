-- Copyright 2016 The Cartographer Authors
--
-- Cartographer 2D SLAM Configuration
-- Optimized for: Jetson Nano + LD19 LiDAR + Differential Drive Robot
-- Robot specs: 0.3 m/s max, 21cm wheel separation, indoor navigation

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "dummy_base_link",      -- Frame to track (must match robot TF)
  published_frame = "dummy_base_link",     -- Frame for pose publishing
  odom_frame = "robot_0/odom",             -- Odometry frame from UKF
  provide_odom_frame = false,              -- Let UKF handle odom→base_link TF (avoid duplicate)
  publish_frame_projected_to_2d = false,   -- 2D mode
  use_odometry = true,                     -- Use UKF odometry
  use_nav_sat = false,                     -- No GPS
  use_landmarks = false,                   -- No landmarks
  num_laser_scans = 1,                     -- LD19 LiDAR
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,     -- NO subdivision (giảm timing issues)
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,      -- CRITICAL FIX: TĂNG từ 0.2 → 0.5s (tránh drop data!)
  submap_publish_period_sec = 0.5,         -- Publish submaps every 0.3s
  pose_publish_period_sec = 0.05,          -- GIẢM từ 0.03 → 0.05 (20Hz thay vì 33Hz, giảm tải)
  trajectory_publish_period_sec = 0.05,    -- GIẢM từ 0.04 → 0.05 (20Hz, match pose)
  rangefinder_sampling_ratio = 1.0,        -- Use all laser scans
  odometry_sampling_ratio = 0.8,           -- Use all odom messages
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 0.0,                -- Disable IMU (use UKF odom instead)
  landmarks_sampling_ratio = 1.0,
}

-- ============================================================================
-- MAP BUILDER (OPTIMIZED for Jetson Nano)
-- ============================================================================
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 1  -- Giảm background threads (tránh optimization)

-- ============================================================================
-- TRAJECTORY BUILDER 2D (SLAM Parameters)
-- ============================================================================
TRAJECTORY_BUILDER_2D.min_range = 0.1                    -- Min laser range (10cm)
TRAJECTORY_BUILDER_2D.max_range = 5.0                    -- Max laser range (5m, match LD19)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false               -- Disable IMU (UKF already has it)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Better matching
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05              -- CRITICAL: GIẢM từ 0.1 → 0.05m (update nhiều hơn khi di chuyển)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.pi / 180 * 0.5  -- CRITICAL: GIẢM từ 1° → 0.5° (update nhiều hơn khi xoay!)

-- Real-time correlative scan matcher (BALANCED - update all directions)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- TĂNG từ 0.08 → 0.15 (match rộng hơn cho backward/side)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- TĂNG từ 15° → 30° (match mọi hướng!)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0   -- GIẢM từ 5.0 → 1.0 (ít phạt, linh hoạt hơn)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0     -- GIẢM từ 10.0 → 1.0 (cho phép update backward)

-- Ceres scan matcher (OPTIMIZED for accuracy)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 4.0   -- GIẢM từ 10.0 về 5.0 (cân bằng)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0     -- GIẢM từ 50.0 về 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60.0        -- GIẢM từ 100.0 về 60.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12  -- GIẢM từ 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4          -- Jetson 4 cores, dùng 2

-- Submaps (INCREASED for better coverage all directions)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120       -- TĂNG từ 90 → 120 (submap lớn hơn, bao phủ mọi hướng)
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 5cm resolution

-- ============================================================================
-- POSE GRAPH OPTIMIZATION (Background Loop Closure)
-- COMPLETELY DISABLED - Tắt hẳn để tránh finish_trajectory làm nát map!
-- ============================================================================
POSE_GRAPH.optimize_every_n_nodes = 0                     -- TẮT optimization trong lúc map
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0       -- CRITICAL FIX: TẮT hẳn constraint builder (0.3 → 0.0)!
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.
POSE_GRAPH.constraint_builder.min_score = 0.99           -- Tăng lên 0.99 (không match được)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.99  -- Tăng lên 0.99

-- Fast correlative scan matcher (for loop closure) - TIGHTENED
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.  -- GIẢM từ 7 → 5m
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)  -- CRITICAL: GIẢM từ 30° → 15°!
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7

-- Ceres scan matcher (for loop closure refinement) - TIGHTENED rotation
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 40.  -- CRITICAL: TĂNG từ 1.0 → 40.0 (phạt nặng rotation error!)
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10  -- GIẢM
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 2

-- Optimization problem - COMPLETELY DISABLED!
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 5e5
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 2e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 3e5
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e3
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 1  -- Tối thiểu (không được = 0!)
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2

-- ============================================================================
-- NOTES
-- ============================================================================
-- OPTIMIZATIONS for Jetson Nano (vs default):
-- - submaps.num_range_data: 90 → 60 (33% less memory)
-- - max_num_iterations: 20 → 12 (40% faster)
-- - num_threads: 1 → 2 (use 2 of 4 cores)
-- - optimize_every_n_nodes: 90 → 60 (less frequent optimization)
-- - sampling_ratio: 0.5 → 0.3 (fewer loop closure checks)
--
-- MAP QUALITY IMPROVEMENTS (v3 - CONSERVATIVE UPDATE):
-- CHIẾN LƯỢC: ÍT CẬP NHẬT HƠN, NHƯNG CHẮC CHẮN HƠN
-- - motion_filter.max_distance: 0.15 → 0.25m (phải di chuyển 25cm mới insert scan)
-- - motion_filter.max_angle: 1.5° → 3.0° (phải xoay 3° mới insert scan)
-- - num_range_data: 60 → 90 (submap lớn hơn, ổn định hơn)
-- - min_score: 0.55 → 0.70 (CHỈ chấp nhận match score ≥ 70%)
-- - global_min_score: 0.60 → 0.75 (loop closure phải ≥ 75%)
-- - optimize_every_n_nodes: 60 → 90 (ít optimize hơn, map ổn định hơn)
-- - occupied_space_weight: 10.0 → 5.0 (cân bằng giữa laser và odometry)
-- - translation_weight: 50 → 20 (linh hoạt hơn)
-- - rotation_weight: 100 → 60 (linh hoạt hơn)
--
-- KẾT QUẢ MONG ĐỢI:
-- - Map update chậm hơn (phải di chuyển 25cm hoặc xoay 3°)
-- - Chỉ ghi nhận scan khi match score ≥ 70%
-- - Map ổn định hơn, ít "nhai" hơn
-- - CPU: 25-30% (giảm do ít insert scan hơn)
--
-- NẾU VẪN "NHAI":
-- - Tăng max_distance lên 0.35m
-- - Tăng max_angle lên 5.0°
-- - Tăng min_score lên 0.75

return options
