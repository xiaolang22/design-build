-- 使用雷达和imu

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 坐标系配置
  map_frame = "map",
  tracking_frame = "imu_link",          -- 使用IMU坐标系进行跟踪
  published_frame = "base_link",        -- 发布基座坐标系位姿
  odom_frame = "odom",                  -- 仍然定义odom坐标系，但Cartographer不使用外部数据
  provide_odom_frame = true,            -- 重要修改：让Cartographer自己发布odom变换
  publish_frame_projected_to_2d = false,

  -- 传感器使用配置
  use_pose_extrapolator = true,         -- 仍然启用位姿推断器，但仅使用IMU和扫描匹配
  use_odometry = false,                 -- 关键修改：禁用外部里程计
  use_nav_sat = false,
  use_landmarks = false,

  -- 传感器数据流配置
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- 超时和发布周期配置
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  -- 数据采样率
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,        -- 这些采样率不影响，因为use_odometry=false
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,             -- IMU仍然100%使用
  landmarks_sampling_ratio = 1.0,
}

-- 地图构建器配置
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4
-- 在cartographer配置中添加地图发布参数
--MAP_BUILDER.publish_frame_rate = 5.0  -- 新增：确保地图发布频率为5Hz

-- 2D轨迹构建器配置
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.min_range = 0.06
TRAJECTORY_BUILDER_2D.max_range = 5.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- IMU配置（更加重要，因为现在没有里程计）
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0

-- 扫描匹配配置（需要更鲁棒，因为没有里程计先验）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- 稍微增大搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(25.)  -- 增大角度搜索
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 在轨迹构建器中调整地图分辨率
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 确保与costmap分辨率一致

-- 运动滤波配置
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.0

-- 位姿图优化配置
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.60  -- 稍微降低，因为匹配可能更难

-- 由于没有外部里程计，调整IMU权重
-- POSE_GRAPH.optimization_problem.imu_rotation_weight = 1e3  -- IMU在旋转估计中权重更高
-- POSE_GRAPH.optimization_problem.imu_acceleration_weight = 1e2  -- 加速度权重

return options