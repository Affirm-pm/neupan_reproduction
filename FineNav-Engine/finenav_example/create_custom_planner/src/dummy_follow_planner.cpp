// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_follow_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numbers>

#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {
namespace {

double squaredDistance(const geometry_msgs::msg::Point& lhs, const geometry_msgs::msg::Point& rhs) {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return dx * dx + dy * dy + dz * dz;
}

double distance(const geometry_msgs::msg::Point& lhs, const geometry_msgs::msg::Point& rhs) {
    return std::sqrt(squaredDistance(lhs, rhs));
}

finenav::common::Position3D positionOf(const geometry_msgs::msg::Pose& pose) {
    return {pose.position.x, pose.position.y, pose.position.z};
}

}  // namespace

void DummyFollowPlanner::configure(const ConfigType& config) {
    linear_speed_ = config.linear_speed;
    angular_speed_ = config.angular_speed;
    lookahead_distance_ = config.lookahead_distance;
}

DummyFollowPlanner::Result DummyFollowPlanner::plan(const Context& context, const IWorldView& world_view) {
    const auto& path = context.ref_traj.poses;
    if (path.empty() || lookahead_distance_ <= 0.0) {
        return std::nullopt;
    }

    const auto& robot_position = context.robot_state.pose.position;
    auto closest = std::min_element(path.begin(), path.end(), [&](const auto& lhs, const auto& rhs) {
        return squaredDistance(lhs.position, robot_position) < squaredDistance(rhs.position, robot_position);
    });

    std::size_t target_index = static_cast<std::size_t>(std::distance(path.begin(), closest));
    double travelled = 0.0;
    while (target_index + 1 < path.size() && travelled < lookahead_distance_) {
        travelled += distance(path[target_index].position, path[target_index + 1].position);
        ++target_index;
    }

    const auto& target = path[target_index];
    if (!world_view.isWalkable(positionOf(target))) {
        return std::nullopt;
    }

    const double dx = target.position.x - robot_position.x;
    const double dy = target.position.y - robot_position.y;
    const double desired_yaw = std::atan2(dy, dx);

    const auto& q = context.robot_state.pose.orientation;
    const double current_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    double yaw_error = desired_yaw - current_yaw;
    while (yaw_error > std::numbers::pi) {
        yaw_error -= 2.0 * std::numbers::pi;
    }
    while (yaw_error < -std::numbers::pi) {
        yaw_error += 2.0 * std::numbers::pi;
    }

    geometry_msgs::msg::Twist command;
    command.linear.x = linear_speed_ * std::max(0.0, 1.0 - std::abs(yaw_error) / 1.5);
    command.angular.z = angular_speed_ * yaw_error;

    finenav::core::OutputTrajectoryT<DummyFollowPlanner> result;
    result.twists.push_back(command);
    return result;
}

}  // namespace finenav::example

FINENAV_REGISTER_PLANNER(finenav::example::DummyFollowPlanner, dummy_follow_planner)
