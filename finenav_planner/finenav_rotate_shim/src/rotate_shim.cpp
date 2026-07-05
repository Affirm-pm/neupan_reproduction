// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_rotate_shim/rotate_shim.hpp"

#include <algorithm>
#include <cmath>

namespace finenav {

// ==============================================================================
// Configure
// ==============================================================================
void RotateShim::configure(const ConfigType& params)
{
  forward_sampling_distance_ = params.forward_sampling_distance;
  angular_speed_             = params.angular_speed;
  angular_dist_threshold_    = params.angular_dist_threshold;
}

// ==============================================================================
// Reset
// ==============================================================================
void RotateShim::reset()
{
  // Nothing persistent to reset between episodes
}

// ==============================================================================
// Plan
// ==============================================================================
std::optional<Trajectory<RotateShim::OutputProfile>> RotateShim::plan(
    const Context& ctx,
    const IMapView& /*map_view*/)
{
  const auto& ref_traj = ctx.ref_traj;

  // 参考轨迹为空则无法旋转，直接退出
  if (ref_traj.poses.empty()) {
    return std::nullopt;
  }

  // 1. 在参考路径上采样前方 sampling_distance 处的点
  const auto sampled_pt = getSampledPathPt(ref_traj.poses, forward_sampling_distance_);

  // 2. 计算采样点在机器人坐标系下的方向
  const double dx = sampled_pt.x - ctx.robot_state.position.x();
  const double dy = sampled_pt.y - ctx.robot_state.position.y();

  // 3. 计算当前朝向
  geometry_msgs::msg::Quaternion q;
  q.w = ctx.robot_state.orientation.w();
  q.x = ctx.robot_state.orientation.x();
  q.y = ctx.robot_state.orientation.y();
  q.z = ctx.robot_state.orientation.z();
  const double current_yaw = yawFromQuat(q);

  // 4. 计算 angular distance：采样点在机器人前方方向角 — 当前朝向
  const double target_yaw = std::atan2(dy, dx);
  const double angular_distance = normalizeAngle(target_yaw - current_yaw);

  // 5. 已对齐 → 返回 nullopt，触发 TrackingPolicy 终止
  if (std::abs(angular_distance) < angular_dist_threshold_) {
    return std::nullopt;
  }

  // 6. 未对齐 — 生成纯旋转轨迹
  Trajectory<RotateShim::OutputProfile> output;
  const double sign = (angular_distance > 0.0) ? 1.0 : -1.0;

  // 仅输出一个点（当前位姿 + 旋转 twist）
  geometry_msgs::msg::Pose p;
  p.position.x = ctx.robot_state.position.x();
  p.position.y = ctx.robot_state.position.y();
  p.position.z = ctx.robot_state.position.z();
  p.orientation.w = ctx.robot_state.orientation.w();
  p.orientation.x = ctx.robot_state.orientation.x();
  p.orientation.y = ctx.robot_state.orientation.y();
  p.orientation.z = ctx.robot_state.orientation.z();
  output.poses.push_back(p);

  geometry_msgs::msg::Twist twist;
  twist.linear.x   = 0.0;
  twist.linear.y   = 0.0;
  twist.linear.z   = 0.0;
  twist.angular.x  = 0.0;
  twist.angular.y  = 0.0;
  twist.angular.z  = sign * angular_speed_;
  output.twists.push_back(twist);

  return output;
}

// ==============================================================================
// getSampledPathPt — 参照 Nav2 RotationShimController::getSampledPathPt
// ==============================================================================
geometry_msgs::msg::Point RotateShim::getSampledPathPt(
    const std::vector<geometry_msgs::msg::Pose>& ref_poses,
    double sampling_distance)
{
  if (ref_poses.size() < 2) {
    // 路径过短，直接返回唯一点的位置
    return ref_poses.front().position;
  }

  const auto& start = ref_poses.front().position;

  // 从第二点开始遍历，累加距离
  for (size_t i = 1; i < ref_poses.size(); ++i) {
    const double dx = ref_poses[i].position.x - start.x;
    const double dy = ref_poses[i].position.y - start.y;
    if (std::hypot(dx, dy) >= sampling_distance) {
      return ref_poses[i].position;
    }
  }

  // 路径不够长，返回最后一个点
  return ref_poses.back().position;
}

// ==============================================================================
// yawFromQuat
// ==============================================================================
double RotateShim::yawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// ==============================================================================
// normalizeAngle
// ==============================================================================
double RotateShim::normalizeAngle(double angle)
{
  while (angle >  M_PI) { angle -= 2.0 * M_PI; }
  while (angle < -M_PI) { angle += 2.0 * M_PI; }
  return angle;
}

}  // namespace finenav
