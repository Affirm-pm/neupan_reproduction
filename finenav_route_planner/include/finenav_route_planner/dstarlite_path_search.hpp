// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <cmath>

#include "finenav_core/plugin_planner.hpp"
#include "finenav_route_planner/dstarlite_params.hpp"

#include "finenav_route_planner/dstarlite/dstarlite_algorithm.hpp"
#include "finenav_route_planner/common/world_view.hpp"

namespace finenav {
using finenav::common::Position3D;
using finenav::common::Region3D;
using finenav::core::IOProfile;
using finenav::core::PlanningContext;
using finenav::core::InputTrajectoryT;
using finenav::core::OutputTrajectoryT;
using finenav::core::Trajectory;

/**
 * @brief D* Lite 增量路径搜索规划器（FineNav 框架适配层）
 * @details 该类是 D* Lite 算法在 FineNav 框架中的 PlannerConcept 实现，
 *          内部持有 dstarlite::DStarLiteAlgorithm 引擎实例并管理其完整生命周期。
 * @details D* Lite 与 A* 的本质区别：
 *          - A* 是无状态的：每次规划从零开始搜索
 *          - D* Lite 是有状态的：在多次规划之间维护 g/rhs 值和优先队列，
 *            地图变化（障碍物更新）时仅重算受影响区域，重规划效率远高于 A*
 * @details 工作流程：
 *          1. 首次调用 plan(): 构建引擎 → initialize() → computeShortestPath() → 提取路径
 *          2. 后续调用 plan(): 同步地图变化(syncMap) → 增量重规划(replan) → 提取路径
 * @note 该类定义了专属的 IWorldView 内部类，在标准 astar::IWorldView 基础上扩展了
 *       rows()/cols()/layers() 栅格维度查询接口，供引擎预分配内部数组使用
 */
class DStarLitePathSearch {
public:
    DStarLitePathSearch() = default;

    // ===== FineNav 框架元数据 =====

    /// 参数类型（由 generate_parameter_library 从 dstarlite.yaml 生成）
    using ConfigType = dstarlite_planner::Params;

    /// 输入端数据规格：目标取 ref_traj 末位姿，故需要 pose
    static constexpr IOProfile InputProfile = {
        .pose  = true,
        .vel   = false,
        .accel = false,
        .time  = false
    };
    /// 输出端数据规格：仅输出位姿序列（路径点）
    static constexpr IOProfile OutputProfile = {
        .pose  = true,
        .vel   = false,
        .accel = false,
        .time  = false
    };

    // ===== D* Lite 专用地图视图接口 =====

    /**
     * @brief D* Lite 专用的地图视图接口
     * @details 在标准 astar::IWorldView（isWalkable / getCost / getStepSize）基础上，
     *          增加了栅格维度查询接口 rows() / cols() / layers()，
     *          用于 D* Lite 引擎预分配 g/rhs/obstacles 等一维状态数组
     * @note 已有的 AStar / BFS / Route 地图适配器不受影响，它们只需实现 astar::IWorldView；
     *       仅当使用 DStarLitePathSearch 时才需要额外提供维度信息
     */
    class IWorldView : public astar::IWorldView {
    public:
        /**
         * @brief 获取栅格行数（y 轴方向）
         * @return 行数
         */
        virtual int rows() const = 0;

        /**
         * @brief 获取栅格列数（x 轴方向）
         * @return 列数
         */
        virtual int cols() const = 0;

        /**
         * @brief 获取栅格层数（z 轴方向）
         * @return 层数，默认返回 1 表示 2-D 平面地图
         */
        virtual int layers() const { return 1; }
    };

    using Context = PlanningContext<DStarLitePathSearch>;
    using Result  = std::optional<OutputTrajectoryT<DStarLitePathSearch>>;

    // ===== PlannerConcept 接口实现 =====

    /**
     * @brief 根据 YAML 生成的参数配置规划器
     * @param params 参数结构体（由 generate_parameter_library 生成）
     * @details 解析连接方式字符串（"4"/"8"/"6"/"18"/"26"）和启发函数字符串
     *          （"Manhattan"/"Euclidean"/"Chebyshev"/"Diagonal"），若连接方式或启发函数
     *          发生变化（结构性变化），则销毁旧引擎触发完全重建
     */
    void configure(const ConfigType& params);

    /**
     * @brief 重置规划器状态（任务边界处调用）
     * @details 销毁当前 D* Lite 引擎，下一次 plan() 时将从头构建新引擎
     */
    void reset();

    /**
     * @brief 执行路径规划
     * @param ctx 规划上下文（包含机器人状态、目标位姿等信息）
     * @param world_view D* Lite 专用地图视图（提供栅格维度 + 可通行性查询）
     * @return 成功时返回路径位姿序列，失败时返回 std::nullopt
     * @details 首次调用：构建引擎 → 初始化 → 搜索 → 提取路径
     *          后续调用：同步地图 → 增量重规划 → 提取路径
     *          若目标点发生变化，自动触发完整的重新初始化
     */
    Result plan(const Context& ctx, const IWorldView& world_view);

private:
    // ===== 配置参数 =====

    astar::Connectivity connectivity_      = astar::Connectivity::N8;    // 邻域连接方式
    astar::Heuristic    heuristic_         = astar::Heuristic::Diagonal; // 启发函数类型
    double goal_tolerance_m_  = 0.5;        // 目标容忍距离（米）
    int    max_expansions_    = 1'000'000;  // 最大扩展节点数
    int    max_path_steps_    = 10'000;     // 最大路径步数
    double max_planning_time_ = 5.0;        // 最大规划时间（秒）
    double movement_weight_   = 1.0;        // 移动代价权重
    double greedy_weight_     = 1.0;        // 启发式贪婪权重
    double terrain_weight_    = 1.0;        // 地形代价乘数

    // ===== 持久状态 =====

    std::unique_ptr<dstarlite::DStarLiteAlgorithm> engine_;  // D* Lite 搜索引擎实例
    double       step_size_    = 1.0;                     // 栅格步长（米/格）

    // ===== 线程安全 =====

    mutable std::mutex mutex_;  // 互斥锁（plan/reset 为写操作，需互斥）
};

}  // namespace finenav
