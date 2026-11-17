#include "global_planner.hpp"
#include <algorithm>
#include <cmath>
#include "finenav_visualizer/marker_viz.hpp"
#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {
void GlobalPlanner::configure(const ConfigType& c) {
    astar_planner::Params params;
    // Four-connected search avoids the upstream algorithm's diagonal corner cutting.
    params.connectivity = "4";
    params.heuristic = "Euclidean";
    params.goal_tolerance = 0.0;
    params.max_iterations = c.max_iterations;
    params.max_planning_time = c.max_planning_time;
    params.movement_weight = 1.0;
    params.greedy_weight = 1.0;
    planner_.configure(params);
    reset();
}

GlobalPlanner::Result GlobalPlanner::plan(const Context& ctx, const IWorldView& world) {
    if (ctx.ref_traj.poses.empty()) return std::nullopt;
    const auto& start = ctx.robot_state.pose;
    const auto& goal = ctx.ref_traj.poses.back();
    const auto finite = [](const auto& p) {
        return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
            std::isfinite(p.position.z) && std::isfinite(p.orientation.x) &&
            std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
            std::isfinite(p.orientation.w);
    };
    const double step = world.getStepSize();
    if (!finite(start) || !finite(goal) || !std::isfinite(step) || step <= 0.0)
        return std::nullopt;
    if (!world.isWalkable({start.position.x, start.position.y, 0.0}) ||
        !world.isWalkable({goal.position.x, goal.position.y, 0.0})) return std::nullopt;
    // The official grid is world-origin anchored. Normalize z for this SE(2) bridge.
    auto state = ctx.robot_state;
    state.pose.position.z = 0.0;
    core::InputTrajectoryT<AstarPathSearch> reference;
    reference.poses.push_back(goal);
    reference.poses.back().position.z = 0.0;
    auto route = planner_.plan({state, reference}, world);
    if (!route || route->poses.empty()) return std::nullopt;

    core::OutputTrajectoryT<GlobalPlanner> result;
    result.poses.push_back(start);
    // Replace endpoint cells, rather than append the exact goal behind its
    // centre. Appending can introduce a tiny backward segment that asks the
    // local optimizer to turn around just before terminal alignment.
    for (size_t i = 1; i + 1 < route->poses.size(); ++i) {
        auto p = route->poses[i];
        p.position.z = 0.0;
        result.poses.push_back(p);
    }
    // All connectors are collision-checked below, including same-cell goals.
    result.poses.push_back(goal);
    for (size_t i = 1; i < result.poses.size(); ++i) {
        const auto& a = result.poses[i - 1].position;
        const auto& b = result.poses[i].position;
        const double distance = std::hypot(b.x - a.x, b.y - a.y);
        if (!std::isfinite(distance) || distance / step > 1000000) return std::nullopt;
        const int count = std::max(1, static_cast<int>(std::ceil(distance / (step * 0.25))));
        for (int j = 0; j <= count; ++j) {
            const double t = static_cast<double>(j) / count;
            if (!world.isWalkable({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), 0.0}))
                return std::nullopt;
        }
        if (i + 1 < result.poses.size()) {
            const auto& next = result.poses[i + 1].position;
            const double yaw = std::atan2(next.y - b.y, next.x - b.x);
            result.poses[i].orientation.z = std::sin(yaw / 2);
            result.poses[i].orientation.w = std::cos(yaw / 2);
        }
    }
    publishPath(result);
    return result;
}

void GlobalPlanner::publishPath(const core::OutputTrajectoryT<GlobalPlanner>& path) {
    std::vector<Eigen::Vector3d> line;
    for (const auto& p : path.poses) line.emplace_back(p.position.x, p.position.y, 0.05);
    auto& channel = viz_.markers("/finenav_global_path");
    channel.addLineStrip(line, 0.06, viz::Color{0.15f, 0.9f, 0.3f});
    viz_.publish(channel);
}
}  // namespace finenav::example

FINENAV_REGISTER_PLANNER(finenav::example::GlobalPlanner, finenav_global_planner)
