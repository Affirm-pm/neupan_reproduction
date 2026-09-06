// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/bfs_path_search.hpp"

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

#include "finenav_core/common/algo_register_macros.hpp"

using finenav::core::Position3D;
using finenav::core::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::OutputTrajectoryT;
namespace finenav {

void BfsPathSearch::configure(const ConfigType& params)
{
    if      (params.connectivity == "4")  connectivity_ = astar::Connectivity::N4;
    else if (params.connectivity == "8")  connectivity_ = astar::Connectivity::N8;
    else if (params.connectivity == "6")  connectivity_ = astar::Connectivity::N6;
    else if (params.connectivity == "18") connectivity_ = astar::Connectivity::N18;
    else                                  connectivity_ = astar::Connectivity::N26;

    goal_tolerance_m_  = params.goal_tolerance;
    max_iterations_    = params.max_iterations;
    max_planning_time_ = params.max_planning_time;
    movement_weight_   = params.movement_weight;
}

BfsPathSearch::Result BfsPathSearch::plan(
    const Context&  ctx,
    const IMapView& map_view)
{
    const double step = map_view.getStepSize();
    if (step <= 0.0 || !std::isfinite(step))
        return std::nullopt;

    auto worldToCell = [step](double x, double y, double z) -> astar::Vec3 {
        return {
            static_cast<int>(std::floor(x / step)),
            static_cast<int>(std::floor(y / step)),
            static_cast<int>(std::floor(z / step))
        };
    };

    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return { (c.x + 0.5) * step,
                 (c.y + 0.5) * step,
                 (c.z + 0.5) * step };
    };

    auto is_walkable = [&](const astar::Vec3& c) -> bool {
        return map_view.isWalkable(cellCentre(c));
    };
    auto get_cost = [&](const astar::Vec3& c) -> double {
        return map_view.getCost(cellCentre(c));
    };

    if (ctx.ref_traj.poses.empty()) return std::nullopt;

    const auto& sp = ctx.robot_state.pose.position;
    const auto& gp = ctx.ref_traj.poses.back().position;

    const astar::Vec3 start = worldToCell(sp.x, sp.y, sp.z);
    const astar::Vec3 goal  = worldToCell(gp.x, gp.y, gp.z);

    // BFS = A* with Heuristic::Zero (h=0 → pure cost-based Dijkstra expansion)
    astar::AStarAlgorithm algo(is_walkable, get_cost);
    algo.setConnectivity(connectivity_);
    algo.setHeuristic(astar::Heuristic::Zero);
    algo.setGoalTolerance(goal_tolerance_m_ / step);
    algo.setMaxIterations(max_iterations_);
    algo.setMaxPlanningTime(max_planning_time_);
    algo.setMovementWeight(movement_weight_);
    // greedy_weight: irrelevant when h=0, leave at default (1.001 * 0 = 0)

    auto cell_path = algo.findPath(start, goal);
    if (!cell_path)
        return std::nullopt;

    OutputTrajectoryT<BfsPathSearch> output;
    output.poses.reserve(cell_path->size());

    for (const auto& cell : *cell_path) {
        const Position3D w = cellCentre(cell);
        geometry_msgs::msg::Pose p;
        p.position.x    = w.x();
        p.position.y    = w.y();
        p.position.z    = w.z();
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    }

    return output;
}

} // namespace finenav

FINENAV_REGISTER_PLANNER(finenav::BfsPathSearch, bfs_planner)

