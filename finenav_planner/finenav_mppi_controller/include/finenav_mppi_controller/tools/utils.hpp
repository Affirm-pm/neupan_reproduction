// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <string>
#include <limits>
#include <memory>
#include <vector>

#include <xtensor/xarray.hpp>
#include <xtensor/xnorm.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "angles/angles.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"


#include "finenav_mppi_controller/models/optimizer_settings.hpp"
#include "finenav_mppi_controller/models/control_sequence.hpp"
#include "finenav_mppi_controller/models/path.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "finenav_mppi_controller/critic_data.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"

namespace mppi::utils
{
using xt::evaluation_strategy::immediate;

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float wz, const builtin_interfaces::msg::Time & stamp, const std::string & frame)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header.frame_id = frame;
  twist.header.stamp = stamp;
  twist.twist.linear.x = vx;
  twist.twist.angular.z = wz;

  return twist;
}

/**
 * @brief Convert data into TwistStamped
 * @param vx X velocity
 * @param vy Y velocity
 * @param wz Angular velocity
 * @param stamp Timestamp
 * @param frame Reference frame to use
 */
inline geometry_msgs::msg::TwistStamped toTwistStamped(
  float vx, float vy, float wz, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame)
{
  auto twist = toTwistStamped(vx, wz, stamp, frame);
  twist.twist.linear.y = vy;

  return twist;
}

/**
 * @brief Convert path to a tensor
 * @param path Path to convert
 * @return Path tensor
 */
inline models::Path toTensor(const nav_msgs::msg::Path & path)
{
  auto result = models::Path{};
  result.reset(path.poses.size());

  for (size_t i = 0; i < path.poses.size(); ++i) {
    result.x(i) = path.poses[i].pose.position.x;
    result.y(i) = path.poses[i].pose.position.y;
    result.yaws(i) = tf2::getYaw(path.poses[i].pose.orientation);
  }

  return result;
}

/**
 * @brief Check if the robot pose is within tolerance to the goal
 * @param pose_tolerance Pose tolerance to use
 * @param robot Pose of robot
 * @param path Path to retreive goal pose from
 * @return bool If robot is within tolerance to the goal
 */
inline bool withinPositionGoalTolerance(
  float pose_tolerance,
  const geometry_msgs::msg::Pose & robot,
  const models::Path & path)
{
  const auto goal_idx = path.x.shape(0) - 1;
  const auto goal_x = path.x(goal_idx);
  const auto goal_y = path.y(goal_idx);

  const auto pose_tolerance_sq = pose_tolerance * pose_tolerance;

  auto dx = robot.position.x - goal_x;
  auto dy = robot.position.y - goal_y;

  auto dist_sq = dx * dx + dy * dy;

  if (dist_sq < pose_tolerance_sq) {
    return true;
  }

  return false;
}

/**
  * @brief normalize
  * Normalizes the angle to be -M_PI circle to +M_PI circle
  * It takes and returns radians.
  * @param angles Angles to normalize
  * @return normalized angles
  */
template<typename T>
auto normalize_angles(const T & angles)
{
  auto && theta = xt::eval(xt::fmod(angles + M_PI, 2.0 * M_PI));
  return xt::eval(xt::where(theta <= 0.0, theta + M_PI, theta - M_PI));
}

/**
  * @brief shortest_angular_distance
  *
  * Given 2 angles, this returns the shortest angular
  * difference.  The inputs and ouputs are of course radians.
  *
  * The result
  * would always be -pi <= result <= pi.  Adding the result
  * to "from" will always get you an equivelent angle to "to".
  * @param from Start angle
  * @param to End angle
  * @return Shortest distance between angles
  */
template<typename F, typename T>
auto shortest_angular_distance(
  const F & from,
  const T & to)
{
  return normalize_angles(to - from);
}

/**
 * @brief Evaluate furthest point idx of data.path which is
 * nearset to some trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of furthest path point reached by a set of trajectories
 */
inline size_t findPathFurthestReachedPoint(const CriticData & data)
{
  const auto traj_x = xt::view(data.trajectories.x, xt::all(), -1, xt::newaxis());
  const auto traj_y = xt::view(data.trajectories.y, xt::all(), -1, xt::newaxis());

  const auto dx = data.path.x - traj_x;
  const auto dy = data.path.y - traj_y;

  const auto dists = dx * dx + dy * dy;

  size_t max_id_by_trajectories = 0, min_id_by_path = 0;
  float min_distance_by_path = std::numeric_limits<float>::max();
  float cur_dist = 0.0f;

  for (size_t i = 0; i < dists.shape(0); i++) {
    min_id_by_path = 0;
    min_distance_by_path = std::numeric_limits<float>::max();
    for (size_t j = 0; j < dists.shape(1); j++) {
      cur_dist = dists(i, j);
      if (cur_dist < min_distance_by_path) {
        min_distance_by_path = cur_dist;
        min_id_by_path = j;
      }
    }
    max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
  }
  return max_id_by_trajectories;
}

/**
 * @brief Evaluate closest point idx of data.path which is
 * nearset to the start of the trajectory in data.trajectories
 * @param data Data to use
 * @return Idx of closest path point at start of the trajectories
 */
inline size_t findPathTrajectoryInitialPoint(const CriticData & data)
{
  // First point should be the same for all trajectories from initial conditions
  const auto dx = data.path.x - data.trajectories.x(0, 0);
  const auto dy = data.path.y - data.trajectories.y(0, 0);
  const auto dists = dx * dx + dy * dy;

  float min_distance_by_path = std::numeric_limits<float>::max();
  size_t min_id = 0;
  for (size_t j = 0; j < dists.shape(0); j++) {
    if (dists(j) < min_distance_by_path) {
      min_distance_by_path = dists(j);
      min_id = j;
    }
  }

  return min_id;
}

/**
 * @brief evaluate path furthest point if it is not set
 * @param data Data to use
 */
inline void setPathFurthestPointIfNotSet(CriticData & data)
{
  if (!data.furthest_reached_path_point) {
    data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
  }
}

/**
 * @brief evaluate path costs
 * @param data Data to use
 */
template <typename MapView>
inline void findPathCosts(
  CriticData & data,
  const MapView& map_view)
{
  const size_t path_segments_count = data.path.x.shape(0) - 1;
  data.path_pts_valid = std::vector<bool>(path_segments_count, false);
  for (unsigned int idx = 0; idx < path_segments_count; idx++) {
    const auto path_x = data.path.x(idx);
    const auto path_y = data.path.y(idx);
//    if (!costmap->worldToMap(path_x, path_y, map_x, map_y)) {   //TODO：应该做个检查是否在地图内
//      (*data.path_pts_valid)[idx] = false;
//      continue;
//    }

    switch (map_view.getCost(Position3D{static_cast<double>(path_x),                //根据finenav—engine的要求，坐标均为全局坐标，即fixed frame坐标系下的坐标
                                                    static_cast<double>(path_y),
                                                    0.0})) {
      // 使用具体数值替代 nav2_costmap_2d 宏常量：
      // NO_INFORMATION = 255, LETHAL_OBSTACLE = 254, INSCRIBED_INFLATED_OBSTACLE = 253
      case 254: // LETHAL_OBSTACLE
        (*data.path_pts_valid)[idx] = false;
        continue;
      case 253: // INSCRIBED_INFLATED_OBSTACLE
        (*data.path_pts_valid)[idx] = false;
        continue;
      case 255: // NO_INFORMATION
      {
        const bool is_tracking_unknown =
          map_view.isTrackingUnknown();
        (*data.path_pts_valid)[idx] = is_tracking_unknown ? true : false;
        continue;
      }
    }

    (*data.path_pts_valid)[idx] = true;
  }
}

/**
 * @brief evaluate path costs if it is not set
 * @param data Data to use
 */
template <typename MapView>
inline void setPathCostsIfNotSet(
  CriticData & data,
  const MapView& map_view)
{
  if (!data.path_pts_valid) {
    findPathCosts(data, map_view);
  }
}

/**
 * @brief evaluate angle from pose (have angle) to point (no angle)
 * @param pose pose
 * @param point_x Point to find angle relative to X axis
 * @param point_y Point to find angle relative to Y axis
 * @param forward_preference If reversing direction is valid
 * @return Angle between two points
 */
inline float posePointAngle(
  const geometry_msgs::msg::Pose & pose, double point_x, double point_y, bool forward_preference)
{
  float pose_x = pose.position.x;
  float pose_y = pose.position.y;
  float pose_yaw = tf2::getYaw(pose.orientation);

  float yaw = atan2f(point_y - pose_y, point_x - pose_x);

  // If no preference for forward, return smallest angle either in heading or 180 of heading
  if (!forward_preference) {
    return std::min(
      fabs(angles::shortest_angular_distance(yaw, pose_yaw)),
      fabs(angles::shortest_angular_distance(yaw, angles::normalize_angle(pose_yaw + M_PI))));
  }

  return fabs(angles::shortest_angular_distance(yaw, pose_yaw));
}

/**
 * @brief Compare to trajectory points to find closest path point along integrated distances
 * @param vec Vect to check
 * @return dist Distance to look for
 */
inline size_t findClosestPathPt(const std::vector<float> & vec, float dist, size_t init = 0)
{
  auto iter = std::lower_bound(vec.begin() + init, vec.end(), dist);
  if (iter == vec.begin() + init) {
    return 0;
  }
  if (dist - *(iter - 1) < *iter - dist) {
    return iter - 1 - vec.begin();
  }
  return iter - vec.begin();
}

}  // namespace mppi::utils

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__UTILS_HPP_
