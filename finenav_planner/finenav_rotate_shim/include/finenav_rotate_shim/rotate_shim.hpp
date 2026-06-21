// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "finenav_core/concepts.hpp"
#include "finenav_core/plan/data_packet.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_rotate_shim/finenav_rotate_shim_params.hpp"

namespace finenav {

/**
 * @brief RotateShim — 原地旋转规划器，使机器人朝向对准参考路径上的前方采样点。
 *
 * 在 MPPI 追踪之前使用，采样参考路径上 forward_sampling_distance 远处的点
 * 作为旋转目标方向，匀速旋转直到 heading error < angular_dist_threshold。
 *
 * 对齐后 plan() 返回 nullopt，由 TrackingPolicy 终止循环，
 * BT 通过 ForceSuccess 转换后进入 MPPI ComputePlan。
 */
class RotateShim {
public:
  RotateShim() = default;

  using ConfigType = finenav_rotate_shim::Params;
  using IMapView   = nav2_mppi_controller::IMapView;  // 与 MPPI 相同类型，plan() 中忽略
  using Context    = PlanningContext<RotateShim>;

  static constexpr IOProfile InputProfile  = { .pose = true, .vel = false, .accel = false, .time = false };
  static constexpr IOProfile OutputProfile = { .pose = true, .vel = true,  .accel = false, .time = false };

  void configure(const ConfigType& params);
  void reset();

  /**
   * @brief 执行一次旋转规划。
   * @param ctx 包含 robot_state 和 ref_traj
   * @param map_view 忽略（旋转不查地图）
   * @return 若已对齐返回 nullopt；否则返回纯旋转角速度的轨迹
   */
  std::optional<DataPacket<OutputProfile>> plan(const Context& ctx, const IMapView& map_view);

private:
  /**
   * @brief 在参考路径上采样前方 forward_sampling_distance 处的点。
   *
   * 从路径起始点开始累加欧式距离，返回第一个距离 >= forward_sampling_distance 的点。
   * 若路径不够长，返回最后一个点。
   *
   * @param ref_poses 参考轨迹的位姿列表
   * @param sampling_distance 前方采样距离
   * @return 采样点的位姿坐标（x, y, z）
   */
  static geometry_msgs::msg::Point getSampledPathPt(
      const std::vector<geometry_msgs::msg::Pose>& ref_poses,
      double sampling_distance);

  /** @brief 从四元数提取 yaw 角 */
  static double yawFromQuat(const geometry_msgs::msg::Quaternion& q);

  /** @brief 归一化角度到 [-pi, pi] */
  static double normalizeAngle(double angle);

  // ── Parameters ────────────────────────────────────────
  double forward_sampling_distance_ = 0.5;
  double angular_speed_             = 0.8;
  double angular_dist_threshold_    = 0.1;
};

}  // namespace finenav

// Register with generate_parameter_library
FINENAV_REGISTER_ALGO_CONFIG(finenav::RotateShim, finenav_rotate_shim)
