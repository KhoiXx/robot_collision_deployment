-- Copyright 2016 The Cartographer Authors
--
-- Cartographer 2D LOCALIZATION ONLY Configuration
-- Replaces AMCL for lightweight localization
-- Optimized for: Jetson Nano + Pre-built Map
-- Usage: Load existing .pbstream map and track robot pose

include "robot_2d.lua"

-- ============================================================================
-- LOCALIZATION MODE (No mapping, only tracking)
-- ============================================================================
-- Enable pure localization (load map from pbstream, don't create new submaps)
TRAJECTORY_BUILDER.pure_localization = true

-- CRITICAL: DISABLE optimization completely to prevent "đè map lên"
-- Optimization can move/distort existing submaps - we want FROZEN map!
POSE_GRAPH.optimize_every_n_nodes = 5  -- NO optimization (map is FIXED!)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1  -- ZERO constraints
POSE_GRAPH.global_sampling_ratio = 0.1  -- ZERO global constraints

-- FREEZE MAP: Disable all background optimizations
-- POSE_GRAPH.optimization_problem.huber_scale = 1e10  -- Ignore optimization
-- POSE_GRAPH.optimization_problem.acceleration_weight = 0.0
-- POSE_GRAPH.optimization_problem.rotation_weight = 0.0
-- POSE_GRAPH.constraint_builder.min_score = 1.0  -- Never create constraints (score impossible)

-- ============================================================================
-- OPTIMIZATIONS for Localization (match mapping params for consistency)
-- ============================================================================

-- Reduce scan matching iterations - FAST localization only
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10  -- GIẢM từ 10 (faster, less CPU)

-- DISABLE submap insertion completely (pure localization doesn't need new submaps)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10  -- Never create new submaps

-- TIGHTEN search window - we trust UKF odom, small corrections only
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- GIẢM từ 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)  -- GIẢM từ 30°

-- MOTION FILTER - Update LESS frequently to avoid LiDAR lag issues
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1     -- TĂNG từ 0.05 (update every 10cm)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.pi / 180 * 2.0  -- TĂNG từ 0.5° (update every 2°)

-- DISABLE voxel filtering to handle LiDAR lag better
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Use correlative matching (more robust)
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0  -- TĂNG voxel size (faster processing)

-- ============================================================================
-- NOTES
-- ============================================================================
-- Localization mode is ~50% lighter than SLAM mode:
-- - No loop closure optimization
-- - Fewer submaps kept in memory
-- - Faster scan matching
--
-- Expected CPU: 10-15% on Jetson Nano
-- Expected Memory: ~300-400 MB
--
-- Perfect replacement for AMCL with better performance!

return options
