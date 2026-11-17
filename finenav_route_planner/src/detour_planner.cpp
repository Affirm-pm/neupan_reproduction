// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/detour_planner.hpp"

#include <chrono>
#include <cmath>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

#include "finenav_route_planner/common/snap_search_utils.hpp"
#include "finenav_core/algo_register_macros.hpp"

namespace finenav {

using finenav::common::Position3D;
using finenav::common::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::OutputTrajectoryT;

// ── configure ──────────────────────────────────────────────────────────────

void DetourPlanner::configure(const ConfigType& params)
{
    scan_distance_m_      = params.scan_distance_m;
    blockage_threshold_   = params.blockage_threshold;
    reconnect_offset_m_   = params.reconnect_offset_m;
    terrain_weight_       = params.obstacle_cost_weight;
    max_planning_time_    = params.max_planning_time;
}

// ── helpers ─────────────────────────────────────────────────────────────────

namespace {

size_t nearestWaypoint(const std::vector<geometry_msgs::msg::Pose>& poses,
                       double rx, double ry)
{
    size_t best = 0;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < poses.size(); ++i) {
        const double dx = poses[i].position.x - rx;
        const double dy = poses[i].position.y - ry;
        if (dx * dx + dy * dy < best_d2) {
            best_d2 = dx * dx + dy * dy;
            best    = i;
        }
    }
    return best;
}

}  // anonymous namespace

// ── findBlockage ───────────────────────────────────────────────────────────

std::optional<size_t> DetourPlanner::findBlockage(
    const Context& ctx, const IWorldView& world_view, size_t scan_start) const
{
    const auto& poses = ctx.ref_traj.poses;

    // Walk forward up to scan_distance_m; a waypoint is blocked when it is
    // not walkable OR its (normalised) cost reaches blockage_threshold.
    // Same walkability the detour search uses — one source of truth.
    double acc = 0.0;
    for (size_t i = scan_start; i < poses.size(); ++i) {
        if (i > scan_start) {
            acc += std::hypot(poses[i].position.x - poses[i - 1].position.x,
                              poses[i].position.y - poses[i - 1].position.y);
            if (acc > scan_distance_m_) break;
        }

        const Position3D wp{poses[i].position.x, poses[i].position.y, 0.0};
        if (!world_view.isWalkable(wp) || world_view.getCost(wp) >= blockage_threshold_)
            return i;
    }
    return std::nullopt;
}

// ── plan ───────────────────────────────────────────────────────────────────
//
// Output contract: the returned path ALWAYS starts at the robot pose.
// The robot may have been displaced from the reference (manual override,
// tracking error), so every cycle re-anchors: an obstacle-aware connector
// from the robot to the nearest reachable reference waypoint, then the
// reference suffix.  Blockage handling is the same operation with the
// candidate set pushed past the blocked segment.

DetourPlanner::Result DetourPlanner::plan(
    const Context&  ctx,
    const IWorldView& world_view)
{
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    const auto logger = rclcpp::get_logger("DetourPlanner");

    const auto& poses = ctx.ref_traj.poses;
    // No reference trajectory yet (e.g. route_layer has not published).
    // Return empty output to keep the TrackingPolicy loop alive.
    if (poses.empty()) {
        return OutputTrajectoryT<DetourPlanner>{};
    }

    auto passThrough = [&]() {
        OutputTrajectoryT<DetourPlanner> output;
        output.poses = poses;
        return output;
    };

    const double step = world_view.getStepSize();
    if (step <= 0.0) return passThrough();

    const double rx = ctx.robot_state.pose.position.x;
    const double ry = ctx.robot_state.pose.position.y;
    const size_t scan_start = nearestWaypoint(poses, rx, ry);

    // ── 1. Detect blockage ahead of the robot ───────────────────────────────
    const auto blocked_idx = findBlockage(ctx, world_view, scan_start);

    // ── 2. Reconnect candidate set ──────────────────────────────────────────
    // Clear path  → all waypoints from the nearest one onward: the robot may
    //               be off the reference (hijacked), so re-anchor every cycle.
    // Blocked     → additionally push the candidates reconnect_offset_m past
    //               the blocked waypoint.  Candidates still inside the
    //               obstacle are unreachable and skipped by the search.
    size_t cand_begin = blocked_idx.value_or(scan_start);
    if (blocked_idx.has_value()) {
        double acc = 0.0;
        while (cand_begin + 1 < poses.size() && acc < reconnect_offset_m_) {
            acc += std::hypot(poses[cand_begin + 1].position.x - poses[cand_begin].position.x,
                              poses[cand_begin + 1].position.y - poses[cand_begin].position.y);
            ++cand_begin;
        }
    }

    const snap::CellGrid grid{step};
    const astar::Vec3    origin = grid.toCell(rx, ry);

    // ── 2a. Diagnostics only (no behaviour change) ──────────────────────────
    // A lethal start cell makes findPathToAny fail INSTANTLY — that is not a
    // timeout.  Report it as its own failure mode so field logs can tell the
    // modes apart.  Typical causes: the sensor scanning the robot body /
    // passenger, or an over-tight z_obstacle_threshold.
    if (!world_view.isWalkable(grid.centre(origin))) {
        RCLCPP_WARN_THROTTLE(logger, steady_clock, 2000,
            "[Detour] START CELL LETHAL — search cannot even start. "
            "robot=(%.2f, %.2f) exact-pos cost=%.2f, cell-centre cost=%.2f, "
            "nearest wp #%zu is %.2fm away. Pass-through.",
            rx, ry,
            world_view.getCost({rx, ry, 0.0}),
            world_view.getCost(grid.centre(origin)),
            scan_start,
            std::hypot(poses[scan_start].position.x - rx,
                       poses[scan_start].position.y - ry));
        return passThrough();
    }

    if (blocked_idx.has_value()) {
        const auto& bp = poses[*blocked_idx].position;
        const Position3D bw{bp.x, bp.y, 0.0};
        RCLCPP_INFO_THROTTLE(logger, steady_clock, 2000,
            "[Detour] blockage: wp #%zu at (%.2f, %.2f) cost=%.2f walkable=%d | "
            "robot nearest wp #%zu, %.2fm off-path | candidates resume at wp #%zu.",
            *blocked_idx, bp.x, bp.y,
            world_view.getCost(bw), static_cast<int>(world_view.isWalkable(bw)),
            scan_start,
            std::hypot(poses[scan_start].position.x - rx,
                       poses[scan_start].position.y - ry),
            cand_begin);
    }

    std::unordered_map<astar::Vec3, std::vector<size_t>, astar::Vec3Hash> cell_pts;
    int max_depth = 0;
    for (size_t i = cand_begin; i < poses.size(); ++i) {
        const astar::Vec3 c = grid.toCell(poses[i].position.x, poses[i].position.y);
        cell_pts[c].push_back(i);
        max_depth = std::max(max_depth, snap::chebyshev(c, origin));
    }

    // ── 3. Obstacle-aware search to the nearest reachable candidate ─────────
    auto algo = snap::makeSnapSearch(world_view, grid, origin, max_depth,
                                     terrain_weight_, max_planning_time_);
    const auto t0 = std::chrono::steady_clock::now();
    const auto path = algo.findPathToAny(origin,
        [&](const astar::Vec3& c) { return cell_pts.contains(c); });
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    if (!path || path->empty()) {
        // Diagnose which failure mode this is: timeout, or the bounded
        // search space genuinely exhausted (every candidate walled off).
        double nearest_d    = std::numeric_limits<double>::infinity();
        double nearest_cost = -1.0;
        bool   nearest_walk = false;
        for (size_t i = cand_begin; i < poses.size(); ++i) {
            const double d = std::hypot(poses[i].position.x - rx,
                                        poses[i].position.y - ry);
            if (d < nearest_d) {
                const Position3D wp{poses[i].position.x, poses[i].position.y, 0.0};
                nearest_d    = d;
                nearest_cost = world_view.getCost(wp);
                nearest_walk = world_view.isWalkable(wp);
            }
        }
        RCLCPP_WARN_THROTTLE(logger, steady_clock, 2000,
            "[Detour] search FAILED (%s: %.3fs / budget %.2fs) | start cell "
            "cost=%.2f | %zu candidate wps from #%zu, nearest %.2fm away "
            "(cost=%.2f walkable=%d) | blocked_wp=%ld — pass-through.",
            elapsed >= max_planning_time_ * 0.95 ? "TIMEOUT"
                                                 : "search space exhausted",
            elapsed, max_planning_time_,
            world_view.getCost(grid.centre(origin)),
            poses.size() - cand_begin, cand_begin,
            nearest_d, nearest_cost, static_cast<int>(nearest_walk),
            blocked_idx ? static_cast<long>(*blocked_idx) : -1L);
        return passThrough();
    }

    // Earliest waypoint in the goal cell → rejoin the path as early as possible.
    const size_t exit_idx = cell_pts.at(path->back()).front();

    // ── 4. Stitch: [connector: robot → exit waypoint] + [ref suffix] ────────
    geometry_msgs::msg::Pose robot_pose;
    robot_pose.position.x = rx;
    robot_pose.position.y = ry;
    robot_pose.position.z = ctx.robot_state.pose.position.z;
    robot_pose.orientation.w = 1.0;
    const auto connector = snap::buildConnector(
        robot_pose, *path, grid,
        {poses[exit_idx].position.x, poses[exit_idx].position.y, 0.0});

    OutputTrajectoryT<DetourPlanner> output;
    output.poses.reserve(connector.size() + (poses.size() - exit_idx));

    // Drop connector.back(): it coincides with poses[exit_idx], which is
    // appended below with its original orientation intact.
    for (size_t i = 0; i + 1 < connector.size(); ++i) {
        geometry_msgs::msg::Pose p;
        p.position.x    = connector[i].x;
        p.position.y    = connector[i].y;
        p.orientation.w = 1.0;
        output.poses.push_back(p);
    }
    // Skip the exit waypoint if the robot is standing exactly on it —
    // avoids a duplicated leading point.
    size_t suffix_begin = exit_idx;
    if (!output.poses.empty() &&
        std::hypot(output.poses.back().position.x - poses[exit_idx].position.x,
                   output.poses.back().position.y - poses[exit_idx].position.y) < 1e-6)
        ++suffix_begin;
    for (size_t i = suffix_begin; i < poses.size(); ++i) {
        output.poses.push_back(poses[i]);
    }

    if (blocked_idx.has_value()) {
        RCLCPP_INFO_THROTTLE(logger, steady_clock, 2000,
            "[Detour] blockage at waypoint %zu | detour %zu points → rejoin at "
            "waypoint %zu | %zu total.",
            *blocked_idx, connector.size(), exit_idx, output.poses.size());
    } else {
        RCLCPP_INFO_THROTTLE(logger, steady_clock, 5000,
            "[Detour] path clear — re-anchored at robot (rejoin waypoint %zu, "
            "%zu total).",
            exit_idx, output.poses.size());
    }

    return output;
}

} // namespace finenav

FINENAV_REGISTER_PLANNER(finenav::DetourPlanner, detour_planner)
