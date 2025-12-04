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
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,  -- Keep only recent submaps for localization
}

-- Disable global SLAM (loop closure) in localization mode
POSE_GRAPH.optimize_every_n_nodes = 0  -- Disable optimization (no new constraints)

-- ============================================================================
-- OPTIMIZATIONS for Localization (even lighter than SLAM)
-- ============================================================================

-- Reduce scan matching iterations (map is fixed, faster convergence)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 8  -- GIẢM từ 12

-- Reduce submap data (not building map)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30  -- GIẢM từ 60

-- Tighter search window (we know roughly where we are)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05  -- 5cm
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)  -- 10 degrees

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
