// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_line_planner.hpp"

#include <cmath>

#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {
namespace {

// Keep accidental tiny resolutions from monopolizing the planning worker or memory.
constexpr size_t kMaxPathSamples = 10000;

finenav::common::Position3D positionOf(const geometry_msgs::msg::Pose& pose) {
    return {pose.position.x, pose.position.y, pose.position.z};
}

bool isFinite(const geometry_msgs::msg::Pose& pose) {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
           std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) &&
           std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
}

}  // namespace

DummyLinePlanner::Result DummyLinePlanner::plan(const Context& context, const IWorldView& world_view) {
    if (context.ref_traj.poses.empty() || !std::isfinite(path_resolution_) || path_resolution_ <= 0.0) {
        return std::nullopt;
    }

    const auto& start = context.robot_state.pose;
    const auto& goal = context.ref_traj.poses.back();
    if (!isFinite(start) || !isFinite(goal)) {
        return std::nullopt;
    }
    const double dx = goal.position.x - start.position.x;
    const double dy = goal.position.y - start.position.y;
    const double dz = goal.position.z - start.position.z;
    const double distance = std::hypot(dx, dy, dz);
    if (!std::isfinite(distance)) {
        return std::nullopt;
    }
    const double interior_sample_count = std::ceil(distance / path_resolution_);
    if (interior_sample_count > static_cast<double>(kMaxPathSamples - 1)) {
        return std::nullopt;
    }

    finenav::core::OutputTrajectoryT<DummyLinePlanner> result;
    if (distance == 0.0) {
        if (!world_view.isWalkable(positionOf(goal))) {
            return std::nullopt;
        }
        result.poses.push_back(goal);
        return result;
    }

    for (double travelled = 0.0; travelled < distance; travelled += path_resolution_) {
        const double ratio = travelled / distance;
        auto point = start;
        point.position.x += dx * ratio;
        point.position.y += dy * ratio;
        point.position.z += dz * ratio;
        if (!world_view.isWalkable(positionOf(point))) {
            return std::nullopt;
        }
        result.poses.push_back(point);
    }

    if (!world_view.isWalkable(positionOf(goal))) {
        return std::nullopt;
    }
    result.poses.push_back(goal);
    return result;
}

}  // namespace finenav::example

FINENAV_REGISTER_PLANNER(finenav::example::DummyLinePlanner, dummy_line_planner)
