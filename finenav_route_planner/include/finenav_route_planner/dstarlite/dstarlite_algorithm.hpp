// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <vector>

#include "finenav_route_planner/common/vec3.hpp"
#include "finenav_route_planner/common/grid_search_utils.hpp"   // Connectivity, getNeighbors, calculateStepCost
#include "finenav_route_planner/common/heuristic.hpp"            // Heuristic, calculateHeuristic
#include "finenav_route_planner/common/priority_key.hpp"         // Key, kInf

namespace dstarlite {

// ── 从 astar 引入通用栅格搜索类型 ──────────────────────────────────
using astar::Vec3;
using astar::Vec3Hash;
using astar::Heuristic;
using astar::Connectivity;
using astar::calculateHeuristic;
using astar::getNeighbors;
using astar::calculateStepCost;
using finenav::Key;
using finenav::kInf;
using finenav::kKeyEpsilon;

/**
 * @brief 有状态的 D* Lite 增量搜索引擎，支持 2-D / 3-D 栅格地图
 * @details D* Lite 是一种增量启发式搜索算法，与 A* 的核心区别在于它在多次规划调用之间
 *          维护 g 值、rhs 值、优先队列 U 以及 km 偏移量。当地图发生变化（如发现新障碍物）
 *          时，仅需重新计算受影响区域的状态，重规划复杂度为 O(受影响区域) 而非 O(全局)。
 * @details 算法维护三个核心数据结构：
 *          - g(s):  节点 s 当前已知的最短路径代价（从 s 到目标）
 *          - rhs(s): 节点 s 的一步前瞻代价 = min_{s'∈Succ(s)}(cost(s,s') + g(s'))
 *          - U:     按 Key 排序的优先队列（最小堆），存放所有不一致节点
 * @details 节点的三种状态：
 *          - 一致(consistent):     g(s) == rhs(s)，节点处于最优状态
 *          - 过一致(over-consistent): g(s) > rhs(s)，代价降低（如障碍物被清除）
 *          - 欠一致(under-consistent): g(s) < rhs(s)，代价升高（如出现新障碍物）
 * @note 邻域枚举、步长代价等底层操作委托给 grid_search_utils.hpp 中的共享函数，
 *       避免了在 D* Lite 中重复实现
 * @note 该类独立于 FineNav 框架，可在任何栅格地图场景下直接使用
 */
class DStarLiteAlgorithm {
public:
    /// 可通行性回调：传入栅格坐标，返回该格是否可通行
    using IsWalkableFn = std::function<bool(const Vec3& p)>;
    /// 地形代价回调：传入栅格坐标，返回该格的额外通行代价（≥0）
    using GetCostFn    = std::function<double(const Vec3& p)>;

    // ===== 构造 =====

    /**
     * @brief 构造 D* Lite 搜索引擎
     * @param rows 栅格行数（y 轴）
     * @param cols 栅格列数（x 轴）
     * @param is_walkable 可通行性查询回调
     * @param get_cost 地形代价查询回调
     * @param connectivity 邻域连接方式，默认 8 邻域（2-D）
     * @param heuristic 启发函数类型，默认 Diagonal
     * @param layers 栅格层数（z 轴），默认 1 表示 2-D 平面
     * @throw std::invalid_argument 若 rows/cols/layers 非正数
     */
    DStarLiteAlgorithm(int rows, int cols,
                   IsWalkableFn is_walkable,
                   GetCostFn    get_cost,
                   Connectivity connectivity = Connectivity::N8,
                   Heuristic    heuristic    = Heuristic::Diagonal,
                   int          layers       = 1);

    // ===== 栅格查询（转发到回调函数）=====

    /**
     * @brief 查询指定栅格是否可通行
     * @param p 栅格坐标
     * @return 可通行返回 true，否则返回 false
     */
    bool   isWalkable(const Vec3& p) const;

    /**
     * @brief 查询指定栅格的地形代价
     * @param p 栅格坐标
     * @return 地形代价（≥0）
     */
    double getCost(const Vec3& p)    const;

    // ===== 维度信息 =====

    /// 行数（y 轴方向）
    int rows()   const noexcept { return rows_; }
    /// 列数（x 轴方向）
    int cols()   const noexcept { return cols_; }
    /// 层数（z 轴方向），默认 1 表示 2-D 地图
    int layers() const noexcept { return layers_; }

    // ===== 连接方式 / 启发函数（修改后需重建引擎）=====

    /**
     * @brief 设置邻域连接方式
     * @param c 连接方式（N4, N8, N6, N18, N26）
     * @note 修改后不影响已初始化的状态，外部需重新调用 initialize()
     */
    void setConnectivity(Connectivity c);

    /**
     * @brief 设置启发函数类型
     * @param h 启发函数类型（Manhattan, Euclidean, Diagonal, Chebyshev）
     * @note 修改后不影响已初始化的状态，外部需重新调用 initialize()
     */
    void setHeuristic(Heuristic h);

    Connectivity connectivity()  const noexcept { return connectivity_; }
    Heuristic    heuristicType() const noexcept { return heuristic_type_; }

    // ===== 障碍物修改（调用后需执行 replan）=====

    /**
     * @brief 将指定栅格标记为障碍物
     * @param p 栅格坐标
     * @note 设置后该格及所有相邻格的状态会被更新，下次 replan() 时生效
     */
    void setObstacle(const Vec3& p);

    /**
     * @brief 清除指定栅格的障碍物标记
     * @param p 栅格坐标
     * @note 清除后该格及所有相邻格的状态会被更新，下次 replan() 时生效
     */
    void removeObstacle(const Vec3& p);

    // ===== 地图同步 =====

    /**
     * @brief 全量同步外部地图变化
     * @details 遍历全部栅格，重新读取 isWalkable 回调，对比内部障碍物记录。
     *          对状态发生变化的栅格（通行→阻塞 / 阻塞→通行）更新其邻居顶点，
     *          使其进入优先队列等待重规划
     * @note 在外部地图发生变化后、调用 replan() 之前调用此函数
     */
    void syncMap();

    /**
     * @brief 局部同步外部地图变化（高效版本）
     * @param around 搜索中心点（栅格坐标）
     * @param radius_cells Chebyshev 距离搜索半径（栅格数）
     * @details 仅扫描以 around 为中心、radius_cells 为半径的区域，
     *          适用于大地图且变化局部的场景，比全量 syncMap() 开销小得多
     */
    void syncMapAround(const Vec3& around, int radius_cells);

    // ===== 算法生命周期 =====

    /**
     * @brief 初始化（或重新初始化）全部内部状态
     * @param start 起点栅格坐标
     * @param goal 目标点栅格坐标
     * @details 执行步骤：
     *          1. 同步障碍物信息（从回调函数全量读取）
     *          2. 清空 g 值、rhs 值（设为无穷大）
     *          3. 清空优先队列 U 和 key 缓存
     *          4. 若目标被阻塞，在 goal_tolerance 范围内搜索最近的可通行格
     *          5. 将 goal 的 rhs 设为 0，插入优先队列
     * @note 在首次 computeShortestPath() 之前必须调用一次
     */
    void initialize(const Vec3& start, const Vec3& goal);

    /**
     * @brief 主搜索循环 —— 扩展节点直到起点达到一致且最优
     * @details 循环从优先队列 U 中弹出 Key 最小的节点进行扩展：
     *          - 若节点过一致(g > rhs)：将 g 设为 rhs，传播给前驱节点
     *          - 若节点欠一致(g < rhs)：将 g 设为无穷大，更新自身及前驱节点
     *          - 终止条件：起点的 Key 不大于队首 Key 且起点一致(g == rhs)
     * @note 调用 initialize() 之后、每次 replan() 之后均需调用此函数
     */
    void computeShortestPath();

    /**
     * @brief 障碍物变化后的增量重规划
     * @param new_start 新的起点栅格坐标（通常为机器人当前位置）
     * @details 根据机器人移动距离更新 km 偏移量，然后调用 computeShortestPath()
     */
    void replan(const Vec3& new_start);

    /**
     * @brief 机器人沿最优路径移动一步
     * @return 若起点已到达目标返回 true，无可移动邻居返回 false
     * @note 仅更新内部起点位置，不更新 km（符合论文规范）
     */
    bool step();

    // ===== 路径提取 =====

    /**
     * @brief 从当前搜索状态提取最优路径
     * @return 从起点到目标的栅格坐标序列；若目标不可达则返回空向量
     * @details 从起点出发，每一步选择使 (moveCost + g) 最小的邻居，
     *          直到到达目标或达到 max_path_steps_ 上限
     */
    std::vector<Vec3> extractPath() const;

    // ===== 访问器 =====

    const Vec3& start() const noexcept { return start_; }
    const Vec3& goal()  const noexcept { return goal_; }

    /**
     * @brief 查询指定节点的 g 值（当前已知的最短路径代价）
     * @param p 栅格坐标
     * @return g 值
     */
    double g(const Vec3& p)  const;

    /**
     * @brief 查询指定节点的 rhs 值（一步前瞻代价）
     * @param p 栅格坐标
     * @return rhs 值
     */
    double rhs(const Vec3& p) const;

    /// 启发式偏移累积量 km（用于处理机器人移动）
    double km()              const noexcept { return km_; }

    // ===== 可配置的搜索限制 =====

    /// 最大节点扩展数，-1 表示无限制
    void setMaxExpansions(int n)    noexcept { max_expansions_    = n; }
    /// 最大路径步数，-1 表示无限制
    void setMaxPathSteps(int n)     noexcept { max_path_steps_    = n; }
    /// 最大规划时间（秒），0.0 表示无限制
    void setMaxPlanningTime(double s) noexcept { max_planning_time_ = s; }
    /// 目标容忍半径（栅格数），0 表示必须精确到达
    void setGoalTolerance(int n)   noexcept { goal_tolerance_    = n; }
    /// 移动代价权重（控制路径长度与地形代价的相对重要性）
    void setMovementWeight(double w) noexcept { movement_weight_  = w; }
    /// 启发式贪婪权重（1.0 = 最优，>1.0 = 更快但可能次优）
    void setGreedyWeight(double w)   noexcept { greedy_weight_    = w; }
    /// 地形代价乘数（放大或缩小地图地形代价的影响）
    void setTerrainWeight(double w)  noexcept { terrain_weight_   = w; }

    int    maxExpansions()    const noexcept { return max_expansions_; }
    int    maxPathSteps()     const noexcept { return max_path_steps_; }
    double maxPlanningTime()  const noexcept { return max_planning_time_; }
    int    goalTolerance()    const noexcept { return goal_tolerance_; }
    double movementWeight()   const noexcept { return movement_weight_; }
    double greedyWeight()     const noexcept { return greedy_weight_; }
    double terrainWeight()    const noexcept { return terrain_weight_; }

    // ===== 调试 =====

    /**
     * @brief 打印当前栅格地图状态（ASCII 可视化）
     * @param path 可选的高亮路径，路径上的格用 '*' 显示
     * @note 仅打印 z=0 层的内容
     */
    void printGrid(const std::vector<Vec3>& path = {}) const;

private:
    // ===== D* Lite 核心函数 =====

    /**
     * @brief 计算节点 s 的优先队列 Key
     * @details Key = [min(g(s), rhs(s)) + h(start, s) * greedy_weight_ + km,
     *                 min(g(s), rhs(s))]
     *          第一分量用于队列排序，第二分量用于打破平局
     */
    Key  calculateKey(const Vec3& s) const;

    /**
     * @brief 更新节点 u 的 rhs 值并维护优先队列
     * @details 若 u 被阻塞则 g/rhs 设为无穷大并从队列移除；
     *          否则 rhs(u) = min_{s∈Succ(u)}(moveCost(u,s) + g(s))
     */
    void updateVertex(const Vec3& u);

    /// 将节点 s 计算 Key 后插入优先队列（或更新已有条目）
    void insertOrUpdate(const Vec3& s);

    // ===== 邻域与代价（委托给 grid_search_utils.hpp）=====

    /// 获取节点 u 的后继节点列表（可通行的相邻格）
    std::vector<Vec3> getSuccessors(const Vec3& u)   const;
    /// 获取节点 u 的前驱节点列表（无向栅格中等同于 getSuccessors）
    std::vector<Vec3> getPredecessors(const Vec3& u) const;
    /// 获取节点 u 的所有邻居（含障碍物，用于障碍物更新时的波及传播）
    std::vector<Vec3> getAllNeighbors(const Vec3& u) const;
    /// 计算从 a 到 b 的移动代价 = 步长代价 × (movement_weight_ + terrain_weight_ × terrain_cost(b))
    double            moveCost(const Vec3& a, const Vec3& b) const;

    // ===== 辅助函数 =====

    /// 将 3-D 栅格坐标映射为 1-D 数组索引: idx = (z * rows + y) * cols + x
    int  idx(const Vec3& p) const noexcept {
        return (p.z * rows_ + p.y) * cols_ + p.x;
    }
    /// 边界检查，越界时抛出 std::out_of_range
    void checkBounds(const Vec3& p) const;

    // ===== 注入的回调函数 =====

    IsWalkableFn is_walkable_;  // 可通行性查询回调
    GetCostFn    get_cost_;     // 地形代价查询回调

    // ===== 栅格参数 =====

    int rows_;                   // 行数（y 轴）
    int cols_;                   // 列数（x 轴）
    int layers_;                 // 层数（z 轴），1 表示 2-D
    Connectivity   connectivity_; // 邻域连接方式
    Heuristic      heuristic_type_; // 启发函数类型

    // ===== D* Lite 核心状态 =====

    std::vector<double> g_;          // g 值：当前已知的最短路径代价
    std::vector<double> rhs_;        // rhs 值：一步前瞻代价
    std::vector<bool>   obstacles_;  // 内部障碍物记录（缓存，避免频繁回调）
    Vec3 start_{0, 0, 0};           // 当前起点
    Vec3 goal_{0, 0, 0};            // 当前目标点
    double km_    = 0.0;             // 启发式偏移累积量
    bool   valid_ = false;           // 是否已完成至少一次 initialize()

    // ===== 搜索限制参数 =====

    int    max_expansions_    = 1'000'000;   // 最大扩展节点数，-1 为无限制
    int    max_path_steps_    = 10'000;      // 最大路径步数，-1 为无限制
    double max_planning_time_ = 5.0;         // 最大规划时间（秒），0.0 为无限制
    int    goal_tolerance_    = 0;           // 目标容忍半径（栅格数），0 表示精确到达
    double movement_weight_   = 1.0;         // 移动代价权重
    double greedy_weight_     = 1.0;         // 启发式贪婪权重（1.0=最优, >1.0=更快）
    double terrain_weight_    = 1.0;         // 地形代价乘数

    // ===== 优先队列 =====

    using PQEntry = std::pair<Key, Vec3>;
    struct PQCompare {
        bool operator()(const PQEntry& a, const PQEntry& b) const noexcept {
            return a.first > b.first;  // 最小堆：Key 小的优先弹出
        }
    };
    std::priority_queue<PQEntry, std::vector<PQEntry>, PQCompare> U_;  // 优先队列（最小堆）
    std::unordered_map<Vec3, Key, Vec3Hash> key_cache_;                // Key 缓存（用于惰性删除）

    // ===== 线程安全 =====

    mutable std::recursive_mutex mutex_;  // 递归互斥锁（部分函数内部嵌套调用）
};

}  // namespace dstarlite
