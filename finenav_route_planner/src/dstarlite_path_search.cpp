// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/dstarlite_path_search.hpp"

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>

#include "finenav_core/algo_register_macros.hpp"

using finenav::common::Position3D;
using finenav::common::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::OutputTrajectoryT;

namespace finenav {

// ===== 配置 =====

void DStarLitePathSearch::configure(const ConfigType& params)
{
    // ── 解析连接方式字符串 → Connectivity 枚举 ──
    astar::Connectivity new_conn = astar::Connectivity::N8;
    if      (params.connectivity == "4")  new_conn = astar::Connectivity::N4;
    else if (params.connectivity == "8")  new_conn = astar::Connectivity::N8;
    else if (params.connectivity == "6")  new_conn = astar::Connectivity::N6;
    else if (params.connectivity == "18") new_conn = astar::Connectivity::N18;
    else if (params.connectivity == "26") new_conn = astar::Connectivity::N26;

    // ── 解析启发函数字符串 → Heuristic 枚举 ──
    astar::Heuristic new_heur = astar::Heuristic::Diagonal;
    if      (params.heuristic == "Manhattan") new_heur = astar::Heuristic::Manhattan;
    else if (params.heuristic == "Euclidean") new_heur = astar::Heuristic::Euclidean;
    else if (params.heuristic == "Chebyshev") new_heur = astar::Heuristic::Chebyshev;

    // ── 读取数值参数 ──
    goal_tolerance_m_  = params.goal_tolerance;
    max_expansions_    = params.max_expansions;
    max_path_steps_    = params.max_path_steps;
    max_planning_time_ = params.max_planning_time;
    movement_weight_   = params.movement_weight;
    greedy_weight_     = params.greedy_weight;
    terrain_weight_    = params.terrain_weight;

    // ── 检测结构性变化（连接方式/启发函数变更需重建引擎）──
    bool structural_change = false;
    if (engine_) {
        if (new_conn != connectivity_ || new_heur != heuristic_) {
            structural_change = true;
            engine_.reset();  // 销毁旧引擎，下次 plan() 时将完全重建
        }
    }

    connectivity_ = new_conn;
    heuristic_    = new_heur;

    // ── 将数值参数推送到已有引擎（非结构性变化时生效）──
    if (engine_) {
        engine_->setMaxExpansions(max_expansions_);
        engine_->setMaxPathSteps(max_path_steps_);
        engine_->setMaxPlanningTime(max_planning_time_);
        engine_->setGoalTolerance(
            static_cast<int>(std::ceil(goal_tolerance_m_ / step_size_)));
        engine_->setMovementWeight(movement_weight_);
        engine_->setGreedyWeight(greedy_weight_);
        engine_->setTerrainWeight(terrain_weight_);
    }

    (void)structural_change;
}

void DStarLitePathSearch::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    engine_.reset();  // 销毁引擎，下次 plan() 触发热启动
}

// ===== 规划 =====

DStarLitePathSearch::Result DStarLitePathSearch::plan(
    const Context&  ctx,
    const IWorldView& world_view)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // ── 步长校验 ──
    const double step = world_view.getStepSize();
    if (step <= 0.0 || !std::isfinite(step))
        return std::nullopt;

    step_size_ = step;

    if (ctx.ref_traj.poses.empty())
        return std::nullopt;

    // ── 世界坐标 ↔ 栅格坐标转换（与 AstarPathSearch 保持一致）──
    auto worldToCell = [step](double x, double y, double z) -> astar::Vec3 {
        return {static_cast<int>(std::floor(x / step)),
                static_cast<int>(std::floor(y / step)),
                static_cast<int>(std::floor(z / step))};
    };

    auto cellCentre = [step](const astar::Vec3& c) -> Position3D {
        return {(c.x + 0.5) * step,
                (c.y + 0.5) * step,
                (c.z + 0.5) * step};
    };

    // ── 提取起点/目标栅格坐标 ──
    const auto& sp = ctx.robot_state.pose.position;
    const auto& gp = ctx.ref_traj.poses.back().position;

    astar::Vec3 start_cell = worldToCell(sp.x, sp.y, sp.z);
    astar::Vec3 goal_cell  = worldToCell(gp.x, gp.y, gp.z);

    // ── 从 IWorldView 读取栅格维度 ──
    const int rows   = world_view.rows();
    const int cols   = world_view.cols();
    const int layers = world_view.layers();

    if (rows <= 0 || cols <= 0 || layers <= 0) {
        // 地图未提供维度信息，无法初始化引擎
        return std::nullopt;
    }

    // ── 引擎生命周期管理 ──

    const bool dimensions_changed =
        !engine_ ||
        engine_->rows()   != rows ||
        engine_->cols()   != cols ||
        engine_->layers() != layers;

    const bool goal_changed =
        !engine_ ||
        engine_->goal() != goal_cell;

    if (dimensions_changed) {
        // 情况 1：首次调用或栅格尺寸发生变化 → 完全重建引擎
        auto is_walkable = [&](const astar::Vec3& c) -> bool {
            return world_view.isWalkable(cellCentre(c));
        };
        auto get_cost = [&](const astar::Vec3& c) -> double {
            return world_view.getCost(cellCentre(c));
        };

        engine_ = std::make_unique<dstarlite::DStarLiteAlgorithm>(
            rows, cols, is_walkable, get_cost,
            connectivity_, heuristic_, layers);

        // 推送全部配置参数
        engine_->setMaxExpansions(max_expansions_);
        engine_->setMaxPathSteps(max_path_steps_);
        engine_->setMaxPlanningTime(max_planning_time_);
        engine_->setGoalTolerance(
            static_cast<int>(std::ceil(goal_tolerance_m_ / step_size_)));
        engine_->setMovementWeight(movement_weight_);
        engine_->setGreedyWeight(greedy_weight_);
        engine_->setTerrainWeight(terrain_weight_);

        // 初始化并运行首次搜索
        engine_->initialize(start_cell, goal_cell);
        engine_->computeShortestPath();
    }
    else {
        // 情况 2：引擎已存在且维度未变 → 增量重规划
        engine_->setGoalTolerance(
            static_cast<int>(std::ceil(goal_tolerance_m_ / step_size_)));

        if (goal_changed) {
            // 目标发生变化 → 完全重新初始化（相当于新的搜索任务）
            engine_->initialize(start_cell, goal_cell);
            engine_->computeShortestPath();
        } else {
            // 目标不变 → 纯增量重规划（地图障碍物变化）
            engine_->syncMap();
            engine_->replan(start_cell);
        }
    }

    // ── 提取路径并转换为世界坐标系位姿序列 ──
    auto cell_path = engine_->extractPath();
    if (cell_path.empty())
        return std::nullopt;

    OutputTrajectoryT<DStarLitePathSearch> output;
    output.poses.reserve(cell_path.size());

    for (const auto& cell : cell_path) {
        const Position3D w = cellCentre(cell);
        geometry_msgs::msg::Pose pose;
        pose.position.x    = w.x;
        pose.position.y    = w.y;
        pose.position.z    = w.z;
        pose.orientation.w = 1.0;  // 朝向无约束
        output.poses.push_back(pose);
    }

    return output;
}

}  // namespace finenav

FINENAV_REGISTER_PLANNER(finenav::DStarLitePathSearch, dstarlite_planner)
