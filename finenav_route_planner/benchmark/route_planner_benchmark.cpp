// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/**
 * @file route_planner_benchmark.cpp
 * @brief A* 和 D* Lite 规划器性能基准测试套件
 *
 * @details
 * ==========================================================================
 * 实验总览
 * ==========================================================================
 * 本 benchmark 套件使用 Google Benchmark 库对 finenav_route_planner 中的
 * A* 和 D* Lite 两种路径搜索算法进行系统的性能评估。
 *
 * 实验目标:
 *   -# 测量 A* 在不同网格规模和启发函数下的搜索性能
 *   -# 测量 D* Lite 冷启动（首次 computeShortestPath）的性能
 *   -# ★ 对比 D* Lite 增量 replan 与 A* 全量 replan 的性能差异
 *      （这是 D* Lite 的核心优势场景：环境局部变化时仅修复受影响区域）
 *   -# 分析 movement_weight / greedy_weight / terrain_weight 三个权重参数
 *      对路径选择行为的影响，为调参提供定量依据
 *
 * ==========================================================================
 * 实验环境
 * ==========================================================================
 * 硬件:         Intel Core i7-12700H / 32 GB RAM (或填写实际测试机器)
 * 操作系统:     Ubuntu 22.04 LTS
 * 编译器:       GCC 11.4, C++20, Release 模式 (-O2 / -O3)
 * Google Benchmark: 1.8.x
 * 构建系统:     colcon + ament_cmake
 *
 * @note 本 benchmark 不依赖 ROS 2 运行时，仅链接 Google Benchmark 和
 *       算法核心文件（astar_algorithm.cpp, dstarlite_algorithm.cpp），
 *       可在无 ROS 环境下独立运行。
 *
 * ==========================================================================
 * 构建与运行
 * ==========================================================================
 * 构建:
 * @code
 *   cd ~/FineNav_ws
 *   colcon build --symlink-install --packages-select finenav_route_planner \
 *       --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
 * @endcode
 *
 * 运行全部:
 * @code
 *   ./build/finenav_route_planner/test/benchmark/route_planner_benchmark
 * @endcode
 *
 * 按名称过滤:
 * @code
 *   # 仅 A* 相关
 *   ./build/.../route_planner_benchmark --benchmark_filter="AStar"
 *   # 仅 D* Lite 相关
 *   ./build/.../route_planner_benchmark --benchmark_filter="DStarLite"
 *   # 仅 replan 对比
 *   ./build/.../route_planner_benchmark --benchmark_filter="Replan"
 *   # 仅权重灵敏度
 *   ./build/.../route_planner_benchmark --benchmark_filter="Weight"
 * @endcode
 *
 * 输出到 JSON（供后续脚本分析或画图）:
 * @code
 *   ./build/.../route_planner_benchmark --benchmark_out=results.json \
 *       --benchmark_out_format=json
 * @endcode
 *
 * 常用 Google Benchmark 选项:
 *   --benchmark_repetitions=N      重复 N 次取平均（推荐 5~10）
 *   --benchmark_min_time=N         每个 case 最少运行 N 秒
 *   --benchmark_counters_tabular=true  以表格形式显示自定义计数器
 *   --benchmark_list_tests         仅列出所有 benchmark 名称, 不运行
 *
 * ==========================================================================
 * Benchmark 详细说明
 * ==========================================================================
 *
 * --- 1. BM_AStar_GridScale ---
 * 目的:   测量 A* 在不同网格规模和障碍密度下的扩展性能
 * 网格:   100×100 / 500×500 / 1000×1000
 * 障碍密度: 0% (空地图) / 10% / 30%
 * 配置:   8邻域连通, Euclidean 启发函数, goal_tolerance = 0
 * 路径:   (0,0) → (size-1, size-1) 对角线
 * 指标:   执行时间 (ms) + path_len (路径格点数)
 *
 * --- 2. BM_AStar_Heuristic ---
 * 目的:   比较四种启发函数对搜索效率的影响
 * 网格:   500×500 空地图（无障碍物, 排除障碍分布的干扰）
 * 启发函数: 0=Euclidean / 1=Diagonal / 2=Manhattan / 3=Zero(=Dijkstra)
 * 配置:   8邻域连通, goal_tolerance = 0
 * @note
 *   - Euclidean / Diagonal: 扩展节点少, 速度快
 *   - Manhattan: 在 8 邻域下不是 admissible, 可能返回次优路径
 *   - Zero: 退化为 Dijkstra 算法, 扩展最多, 最慢 — 作为速度下界参考
 *
 * --- 3. BM_DStarLite_InitialPlan ---
 * 目的:   测量 D* Lite 冷启动（首次完整搜索）的性能基线
 * 网格:   100×100 / 300×300 / 500×500
 * 障碍密度: 0% / 10% / 30%
 * 配置:   8邻域连通, Diagonal 启发函数
 * 路径:   (0,0) → (size-1, size-1)
 * @note 冷启动时 D* Lite 需初始化所有节点的 rhs/g 值为 ∞，开销比 A* 大，
 *       这里的测量值用作 replan 实验的基线参考
 *
 * --- 4. BM_ReplanCompare_DStarLite / BM_ReplanCompare_AStar ★ ---
 * 目的:   ★ 核心实验 — 增量 replan 与全量 replan 的性能对比
 * 场景:   300×300 网格, 20% 随机障碍
 * 流程:
 *   1. 完成初始规划 (A*: findPath, D* Lite: computeShortestPath)
 *   2. 在初始路径中段 (1/4 ~ 3/4) 随机选取 1~40 个点，添加为障碍物
 *   3. 执行 replan (D* Lite: replan(), A*: 重新 findPath() 全量搜索)
 *   4. 测量 replan 耗时
 * 配置:   D* Lite 用 Diagonal 启发, A* 用 Euclidean 启发
 * 指标:   执行时间 (μs) + init_len + replan_len + obs_added
 * 预期:   障碍少时 D* Lite 显著快于 A*（仅修复局部受影响区域）；
 *         障碍增多后优势缩小（受影响区域扩大）
 *
 * --- 5. BM_CrossCompare_AStar / BM_CrossCompare_DStarLite ---
 * 目的:   同一网格场景下公平对比 A* 和 D* Lite 初始规划性能
 * 网格:   500×500, 10% 障碍（共享 Fixture 保证相同地图）
 * @note D* Lite 冷启动开销在此可直接观测
 *
 * --- 6a. BM_Weight_MovementWeight ---
 * 目的:   分析 movement_weight 对路径选择的影响
 * 网格:   100×100, 无障碍, 中央有高代价矩形带 (x∈[35,45], y∈[10,89], cost=5.0)
 * 起点→终点: (0,50) → (99,50) — 高代价带横在中间, 迫使算法做取舍
 * movement_weight 取值: 0.1 / 0.5 / 1.0 / 5.0 / 10.0
 * greedy_weight: 固定 1.0
 * 代价公式: step_cost × (movement_weight + 1.0 × map_cost)
 * 指标:   path_len + ★ swamp_cells + total_cost
 * 行为预期:
 *   - mw 小 (0.1):  对 terrain cost 敏感 → 绕路避开高代价带
 *   - mw 大 (10.0): 距离优先 → 直接穿过高代价带
 *
 * --- 6b. BM_Weight_GreedyWeight ---
 * 目的:   分析 greedy_weight 对搜索行为的影响
 * 网格:   同 6a (100×100, 中央高代价带 cost=5.0)
 * greedy_weight 取值: 0.5 / 1.0 / 1.5 / 2.0 / 5.0
 * movement_weight: 固定 1.0
 * @note greedy_weight 只改变启发函数的放大系数 (h × gw), 不改变代价函数,
 *       因此不影响最优性条件（仅影响搜索速度）
 * 行为预期:
 *   - gw < 1.0: heuristic 被压缩 → 偏向 Dijkstra → 慢但保证最优
 *   - gw = 1.0: 标准 A* / D* Lite → 理论最优
 *   - gw > 1.0: heuristic 被放大 → 更快但可能次优
 *
 * --- 6c. BM_Weight_Combined ---
 * 目的:   观察 movement_weight × greedy_weight 的交互效应
 * 网格:   同 6a
 * 参数组合: 9 组关键点 — mw={0.1, 1.0, 10.0} × gw={1, 2, 5}
 *
 * --- 6d. BM_Weight_TerrainWeight ---
 * 目的:   分析 terrain_weight 对路径选择的影响
 * 网格:   100×100, 中央高代价带 raw_cost=10.0 (适度代价, 使 tw 过渡清晰可见)
 * terrain_weight 取值: 0.01 / 0.05 / 0.1 / 0.5 / 1.0 / 5.0
 * movement_weight: 固定 1.0, greedy_weight: 固定 1.0
 * 指标:   path_len + ★ swamp_cells + total_cost
 * 行为预期:
 *   - tw 小 (0.01): terrain 几乎被忽略 → 直线穿过
 *   - tw = 1.0:    默认值 → 应绕过高代价带
 *   - tw 大 (5.0):  terrain 主导决策 → 坚决绕路
 * 调参指引:
 *   - 若路径无视地形代价: 增大 tw
 *   - 若路径绕得太远:     减小 tw
 *   - swamp_cells 从 >0 变为 0 的 tw 值即为"开始绕路"的临界阈值
 *
 * --- 6e. BM_Weight_MWxTW ---
 * 目的:   分析 movement_weight × terrain_weight 在极端代价下的交互
 * 网格:   100×100, 中央高代价带 raw_cost=500.0 (模拟接近不可通行的障碍)
 * 参数组合: 5 组极端组合
 * 预期:
 *   - (mw=0.1, tw=0.01):  距离便宜 + terrain 被忽略 → 穿过
 *   - (mw=0.1, tw=10.0):  距离便宜 + terrain 极重 → 绕路
 *   - (mw=10.0, tw=0.01): 距离昂贵 + terrain 被忽略 → 穿过
 *   - (mw=10.0, tw=10.0): 两者都重 → 平衡竞争
 *   - (mw=1.0,  tw=1.0):  默认基准线
 *
 * ==========================================================================
 * 输出解读指南
 * ==========================================================================
 * Google Benchmark 默认输出三列:
 *   Time        — 实耗 wall-clock time
 *   CPU         — CPU 时间（反映纯计算量, 排除 I/O 等待）
 *   Iterations  — 迭代次数（Google Benchmark 自动决定, 保证统计显著性）
 *
 * 自定义计数器（通过 state.counters[...] 设置, 不影响计时）:
 *   path_len      — 找到的路径长度（格点数）, 验证路径合理性和最优性
 *   swamp_cells   — ★ 路径穿过高代价地带的格点数, 核心行为指标
 *                    =0 表示路径完全绕开, >0 表示部分穿过
 *   total_cost    — 路径上地形代价总和（raw map_cost, 未乘 weight）
 *   init_len      — (replan 实验) 变更前原始路径长度
 *   replan_len    — (replan 实验) 重规划后路径长度
 *   obs_added     — (replan 实验) 实际添加的障碍物数量
 *
 * @note
 *   - Google Benchmark 自动计算均值和标准差, 无需手动统计
 *   - 计数器不影响计时, 仅在输出中显示为附加列
 *   - 网格起点 (0,0) 和终点 (size-1,size-1) 始终可行走（generateObstacles 跳过）
 *   - 随机障碍使用固定种子 (seed=42), 保证相同配置下结果可复现
 *
 * ==========================================================================
 * 已知限制
 * ==========================================================================
 *   -# 当前仅测试 8 邻域 (N8) 连通性; 4 邻域 (N4) 不在 benchmark 覆盖范围
 *   -# 障碍物使用 Mersenne Twister 伪随机生成, 不同种子可能产生不同结果;
 *      同一密度不同种子下路径长度会有波动
 *   -# 权重参数直接通过 setter 传入, 不经过 ROS parameter 系统
 *      （行为与运行时一致, 但跳过了 parameter 回调线程的开销）
 *   -# 仅测量时间, 不测量内存占用或缓存命中率
 *   -# D* Lite replan 中起点固定为 (0,0), 不测试动态起点场景
 *   -# 高代价地形为规则的矩形 band, 不涵盖非规则或渐变地形
 *   -# 权重灵敏度实验仅使用 D* Lite（A* 行为类似, 不再重复）
 *
 * @author Jinghui Wang (wangjinghui@sjtu.edu.cn)
 * @author IWIN-FINS Lab, Shanghai Jiao Tong University
 * @date 2026-07-19
 */

#include <benchmark/benchmark.h>

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "grid_map.hpp"
#include "finenav_route_planner/astar/astar_algorithm.hpp"
#include "finenav_route_planner/dstarlite/dstarlite_algorithm.hpp"

// ===== 辅助函数 =====

void generateObstacles(astar::GridMap& grid, int density_pct, uint64_t seed = 42) {
    if (density_pct <= 0) return;
    std::mt19937_64 rng(seed);
    int w = grid.getWidth(), h = grid.getHeight();
    int n_obs = (w * h) * density_pct / 100;

    std::uniform_int_distribution<int> dist_x(0, w - 1);
    std::uniform_int_distribution<int> dist_y(0, h - 1);

    for (int i = 0; i < n_obs; ++i) {
        int x = dist_x(rng);
        int y = dist_y(rng);
        if ((x == 0 && y == 0) || (x == w - 1 && y == h - 1)) continue;
        grid.setObstacle(x, y, true);
    }
}

// ===== Benchmark Fixture =====

class PlannerBench : public benchmark::Fixture {
protected:
    void SetUp(const benchmark::State& state) override {
        int size        = static_cast<int>(state.range(0));
        int density_pct = static_cast<int>(state.range(1));

        grid_ = std::make_unique<astar::GridMap>(size, size);
        generateObstacles(*grid_, density_pct);
    }

    void TearDown(const benchmark::State&) override {
        grid_.reset();
    }

    auto makeLambdas() const {
        return std::make_pair(
            [this](const astar::Vec3& c) { return grid_->isWalkableCell(c.x, c.y); },
            [this](const astar::Vec3& c) { return grid_->getCostCell(c.x, c.y); }
        );
    }

    std::unique_ptr<astar::GridMap> grid_;
};

// ===== 1. A* — 网格规模扫描 (N8, Euclidean) =====

BENCHMARK_DEFINE_F(PlannerBench, BM_AStar_GridScale)(benchmark::State& state) {
    int size = static_cast<int>(state.range(0));
    auto [iw, gc] = makeLambdas();

    for (auto _ : state) {
        state.PauseTiming();
        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(astar::Connectivity::N8);
        algo.setHeuristic(astar::Heuristic::Euclidean);
        algo.setGoalTolerance(0.0);
        state.ResumeTiming();

        auto path = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        benchmark::DoNotOptimize(path);

        state.counters["path_len"] = path.has_value()
            ? static_cast<double>(path->size()) : 0.0;
    }
    state.SetComplexityN(size);
}
BENCHMARK_REGISTER_F(PlannerBench, BM_AStar_GridScale)
    ->ArgsProduct({{100, 500, 1000}, {0, 10, 30}})
    ->Unit(benchmark::kMillisecond)
    ->Complexity();

// ===== 2. A* — 启发函数对比 (固定 500² 网格, N8) =====

static void BM_AStar_Heuristic(benchmark::State& state) {
    int size = 500;
    astar::GridMap grid(size, size);

    auto is_walkable = [&grid](const astar::Vec3& c) {
        return grid.isWalkableCell(c.x, c.y);
    };
    auto get_cost = [&grid](const astar::Vec3& c) {
        return grid.getCostCell(c.x, c.y);
    };

    astar::Heuristic heur;
    switch (state.range(0)) {
        case 0: heur = astar::Heuristic::Euclidean;  break;
        case 1: heur = astar::Heuristic::Diagonal;   break;
        case 2: heur = astar::Heuristic::Manhattan;  break;
        case 3: heur = astar::Heuristic::Zero;       break;
        default: heur = astar::Heuristic::Euclidean;  break;
    }

    for (auto _ : state) {
        state.PauseTiming();
        astar::AStarAlgorithm algo(is_walkable, get_cost);
        algo.setConnectivity(astar::Connectivity::N8);
        algo.setHeuristic(heur);
        algo.setGoalTolerance(0.0);
        state.ResumeTiming();

        auto path = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        benchmark::DoNotOptimize(path);

        state.counters["path_len"] = path.has_value()
            ? static_cast<double>(path->size()) : 0.0;
    }
}
BENCHMARK(BM_AStar_Heuristic)
    ->DenseRange(0, 3, 1)
    ->ArgName("heuristic")
    ->Unit(benchmark::kMillisecond);

// ===== 3. D* Lite — 冷启动初始规划 (computeShortestPath) =====

BENCHMARK_DEFINE_F(PlannerBench, BM_DStarLite_InitialPlan)(benchmark::State& state) {
    int size = static_cast<int>(state.range(0));
    auto [iw, gc] = makeLambdas();

    for (auto _ : state) {
        state.PauseTiming();
        dstarlite::DStarLiteAlgorithm engine(size, size, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.initialize({0, 0, 0}, {size - 1, size - 1, 0});
        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        state.counters["path_len"] = static_cast<double>(path.size());
    }
    state.SetComplexityN(state.range(0));
}
BENCHMARK_REGISTER_F(PlannerBench, BM_DStarLite_InitialPlan)
    ->ArgsProduct({{100, 300, 500}, {0, 10, 30}})
    ->Unit(benchmark::kMillisecond)
    ->MinTime(0.5)
    ->Complexity();

// ===== 4. ★ D* Lite vs A* — 路径被新障碍阻断后的 replan 对比 =====
//
// 场景: 300×300 网格, 20% 随机障碍 → 初始规划
//       然后在已找到的路径上添加 1~40 个新障碍
//       D* Lite: replan() 增量修复受影响区域
//       A*:      重新 findPath() 全量搜索
//       这是 D* Lite 应该展现增量优势的核心场景

// --- D* Lite replan ---
static void BM_ReplanCompare_DStarLite(benchmark::State& state) {
    int size      = 300;
    int n_new_obs = static_cast<int>(state.range(0));

    for (auto _ : state) {
        state.PauseTiming();

        astar::GridMap grid(size, size);
        generateObstacles(grid, 20, static_cast<uint64_t>(state.iterations()));

        auto iw = [&grid](const astar::Vec3& c) { return grid.isWalkableCell(c.x, c.y); };
        auto gc = [&grid](const astar::Vec3& c) { return grid.getCostCell(c.x, c.y); };

        dstarlite::DStarLiteAlgorithm engine(size, size, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.initialize({0, 0, 0}, {size - 1, size - 1, 0});
        engine.computeShortestPath();
        auto initial_path = engine.extractPath();
        if (initial_path.size() < 3) { state.SkipWithError("No init"); continue; }

        std::mt19937_64 rng(static_cast<uint64_t>(state.iterations()) + 12345);
        std::uniform_int_distribution<size_t> pick(initial_path.size() / 4,
                                                    initial_path.size() * 3 / 4);

        int added = 0;
        for (int i = 0; i < n_new_obs * 3 && added < n_new_obs; ++i) {
            auto& cell = initial_path[pick(rng)];
            engine.setObstacle(cell);
            ++added;
        }

        state.ResumeTiming();

        engine.replan({0, 0, 0});
        auto replanned = engine.extractPath();
        benchmark::DoNotOptimize(replanned);

        state.counters["init_len"] = static_cast<double>(initial_path.size());
        state.counters["replan_len"] = static_cast<double>(replanned.size());
        state.counters["obs_added"] = static_cast<double>(added);
    }
}
BENCHMARK(BM_ReplanCompare_DStarLite)
    ->ArgName("new_obs")
    ->DenseRange(1, 40, 10)
    ->Unit(benchmark::kMicrosecond);

// --- A* full replan ---
static void BM_ReplanCompare_AStar(benchmark::State& state) {
    int size      = 300;
    int n_new_obs = static_cast<int>(state.range(0));

    for (auto _ : state) {
        state.PauseTiming();

        astar::GridMap grid(size, size);
        generateObstacles(grid, 20, static_cast<uint64_t>(state.iterations()));

        auto iw = [&grid](const astar::Vec3& c) { return grid.isWalkableCell(c.x, c.y); };
        auto gc = [&grid](const astar::Vec3& c) { return grid.getCostCell(c.x, c.y); };

        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(astar::Connectivity::N8);
        algo.setHeuristic(astar::Heuristic::Euclidean);
        algo.setGoalTolerance(0.0);

        auto initial_path = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        if (!initial_path || initial_path->size() < 3) { state.SkipWithError("No init"); continue; }

        std::mt19937_64 rng(static_cast<uint64_t>(state.iterations()) + 12345);
        std::uniform_int_distribution<size_t> pick(initial_path->size() / 4,
                                                    initial_path->size() * 3 / 4);

        int added = 0;
        for (int i = 0; i < n_new_obs * 3 && added < n_new_obs; ++i) {
            auto& cell = (*initial_path)[pick(rng)];
            grid.setObstacle(cell.x, cell.y, true);
            ++added;
        }

        state.ResumeTiming();

        auto replanned = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        benchmark::DoNotOptimize(replanned);

        state.counters["init_len"] = static_cast<double>(initial_path->size());
        state.counters["replan_len"] = replanned.has_value()
            ? static_cast<double>(replanned->size()) : 0.0;
        state.counters["obs_added"] = static_cast<double>(added);
    }
}
BENCHMARK(BM_ReplanCompare_AStar)
    ->ArgName("new_obs")
    ->DenseRange(1, 40, 10)
    ->Unit(benchmark::kMicrosecond);

// ===== 5. 交叉对比 — 500² 网格, 10% 障碍, 同场景公平对比 =====

BENCHMARK_DEFINE_F(PlannerBench, BM_CrossCompare_AStar)(benchmark::State& state) {
    int size = 500;
    auto [iw, gc] = makeLambdas();
    for (auto _ : state) {
        state.PauseTiming();
        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(astar::Connectivity::N8);
        algo.setHeuristic(astar::Heuristic::Euclidean);
        state.ResumeTiming();
        auto path = algo.findPath({0, 0, 0}, {size - 1, size - 1, 0});
        benchmark::DoNotOptimize(path);
        state.counters["path_len"] = path.has_value()
            ? static_cast<double>(path->size()) : 0.0;
    }
}
BENCHMARK_REGISTER_F(PlannerBench, BM_CrossCompare_AStar)
    ->Args({500, 10})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_DEFINE_F(PlannerBench, BM_CrossCompare_DStarLite)(benchmark::State& state) {
    int size = 500;
    auto [iw, gc] = makeLambdas();
    for (auto _ : state) {
        state.PauseTiming();
        dstarlite::DStarLiteAlgorithm engine(size, size, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.initialize({0, 0, 0}, {size - 1, size - 1, 0});
        state.ResumeTiming();
        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);
        state.counters["path_len"] = static_cast<double>(path.size());
    }
}
BENCHMARK_REGISTER_F(PlannerBench, BM_CrossCompare_DStarLite)
    ->Args({500, 10})
    ->Unit(benchmark::kMillisecond);

// ===== 6. 权重灵敏度分析 =====
//
// 网格: 100×100, 无障碍物, 但中央有一条高代价地形带 (cost=5.0)
// 规划器必须选择: 穿过昂贵地形（路径短）还是绕路（路径长但代价低）
//
// 这是调参的典型场景:
//   - movement_weight  控制距离 vs 地形代价的权衡
//   - greedy_weight    控制搜索贪心程度（可能影响最优性）
//   - terrain_weight   控制地图代价的缩放系数

// --- 6a. movement_weight 扫描 ---

static void BM_Weight_MovementWeight(benchmark::State& state) {
    const int W = 100, H = 100;
    double mw = state.range(0) * 0.1;  // mw_x10 → movement_weight

    for (auto _ : state) {
        state.PauseTiming();

        // Build grid with a high-cost band (x ∈ [35,45], y ∈ [10,89])
        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 5.0;

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        dstarlite::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(mw);
        engine.setGreedyWeight(1.0);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        // Count cells in the high-cost band
        int swamp_cells = 0;
        double total_terrain_cost = 0.0;
        for (const auto& c : path) {
            total_terrain_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_terrain_cost;
    }
}
BENCHMARK(BM_Weight_MovementWeight)
    ->ArgName("mw_x10")
    ->Arg(1)      // mw=0.1  terrain-sensitive — avoid high-cost at all costs
    ->Arg(5)      // mw=0.5  slightly terrain-averse
    ->Arg(10)     // mw=1.0  default
    ->Arg(50)     // mw=5.0  distance-preferring
    ->Arg(100)    // mw=10.0 distance-first — aggressively cut through
    ->Unit(benchmark::kMillisecond);

// --- 6b. greedy_weight 扫描 ---

static void BM_Weight_GreedyWeight(benchmark::State& state) {
    const int W = 100, H = 100;
    double gw = state.range(0) * 0.1;  // gw_x10 → greedy_weight

    for (auto _ : state) {
        state.PauseTiming();

        // Same high-cost band setup
        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 5.0;

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        dstarlite::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(1.0);
        engine.setGreedyWeight(gw);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        int  swamp_cells = 0;
        double total_cost = 0.0;
        for (const auto& c : path) {
            total_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_cost;
    }
}
BENCHMARK(BM_Weight_GreedyWeight)
    ->ArgName("gw_x10")
    ->Arg(5)     // gw=0.5  deflated heuristic — Dijkstra-like, slow but safe
    ->Arg(10)    // gw=1.0  default optimal
    ->Arg(15)    // gw=1.5  slightly greedy — bounded suboptimal
    ->Arg(20)    // gw=2.0  more greedy — may cut corners
    ->Arg(50)    // gw=5.0  very greedy — aggressive focus toward start
    ->Unit(benchmark::kMillisecond);

// --- 6c. 联合扫描: movement_weight × greedy_weight 关键组合 ---

static void BM_Weight_Combined(benchmark::State& state) {
    const int W = 100, H = 100;
    double mw = state.range(0) * 0.1;   // mw_x10 → actual movement_weight
    double gw = state.range(1);          // greedy_weight (integer)

    for (auto _ : state) {
        state.PauseTiming();

        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 5.0;

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        dstarlite::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(mw);
        engine.setGreedyWeight(gw);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        int  swamp_cells = 0;
        double total_cost = 0.0;
        for (const auto& c : path) {
            total_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_cost;
    }
}
BENCHMARK(BM_Weight_Combined)
    // (mw_x10, gw) pairs → actual mw = mw_x10/10
    ->Args({1, 1})      // mw=0.1 terrain-sensitive,  gw=1.0 optimal
    ->Args({10, 1})     // mw=1.0 default,             gw=1.0 optimal
    ->Args({100, 1})    // mw=10.0 distance-first,      gw=1.0 optimal
    ->Args({1, 2})      // mw=0.1 terrain-sensitive,  gw=2.0 faster
    ->Args({10, 2})     // mw=1.0 default,             gw=2.0 faster
    ->Args({100, 2})    // mw=10.0 distance-first,      gw=2.0 faster
    ->Args({1, 5})      // mw=0.1 terrain-sensitive,  gw=5.0 aggressive
    ->Args({10, 5})     // mw=1.0 default,             gw=5.0 aggressive
    ->Args({100, 5})    // mw=10.0 distance-first,      gw=5.0 aggressive
    ->ArgNames({"mw_x10", "gw"})
    ->Unit(benchmark::kMillisecond);

// --- 6d. terrain_weight 扫描 — 地图代价缩放如何改变路径选择 ---

static void BM_Weight_TerrainWeight(benchmark::State& state) {
    const int W = 100, H = 100;
    double tw = state.range(0) * 0.01;  // tw_x100 → terrain_weight

    for (auto _ : state) {
        state.PauseTiming();

        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 10.0;  // moderate raw cost so tw transition is visible

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        dstarlite::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(1.0);
        engine.setGreedyWeight(1.0);
        engine.setTerrainWeight(tw);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        int  swamp_cells = 0;
        double total_cost = 0.0;
        for (const auto& c : path) {
            total_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_cost;
    }
}
BENCHMARK(BM_Weight_TerrainWeight)
    ->ArgName("tw_x100")
    ->Arg(1)      // tw=0.01   terrain nearly ignored → straight through
    ->Arg(5)      // tw=0.05   terrain barely matters
    ->Arg(10)     // tw=0.1    transition zone
    ->Arg(50)     // tw=0.5    somewhat terrain-averse
    ->Arg(100)    // tw=1.0    default — should go around
    ->Arg(500)    // tw=5.0    terrain dominates
    ->Unit(benchmark::kMillisecond);

// --- 6e. movement_weight × terrain_weight 联合扫描 — "距离 vs 地图" 双旋钮 ---

static void BM_Weight_MWxTW(benchmark::State& state) {
    const int W = 100, H = 100;
    double mw = state.range(0) * 0.1;   // mw_x10 → movement_weight
    double tw = state.range(1) * 0.01;  // tw_x100 → terrain_weight

    for (auto _ : state) {
        state.PauseTiming();

        std::vector<double> terrain_cost(W * H, 0.0);
        const int band_x0 = 35, band_x1 = 45;
        const int band_y0 = 10, band_y1 = 89;
        for (int y = band_y0; y <= band_y1; ++y)
            for (int x = band_x0; x <= band_x1; ++x)
                terrain_cost[y * W + x] = 500.0;

        auto iw = [](const astar::Vec3&) { return true; };
        auto gc = [&](const astar::Vec3& c) { return terrain_cost[c.y * W + c.x]; };

        dstarlite::DStarLiteAlgorithm engine(H, W, iw, gc,
                                          astar::Connectivity::N8,
                                          astar::Heuristic::Diagonal);
        engine.setMovementWeight(mw);
        engine.setGreedyWeight(1.0);
        engine.setTerrainWeight(tw);
        engine.initialize({0, 50, 0}, {99, 50, 0});

        state.ResumeTiming();

        engine.computeShortestPath();
        auto path = engine.extractPath();
        benchmark::DoNotOptimize(path);

        int  swamp_cells = 0;
        double total_cost = 0.0;
        for (const auto& c : path) {
            total_cost += terrain_cost[c.y * W + c.x];
            if (c.x >= band_x0 && c.x <= band_x1 &&
                c.y >= band_y0 && c.y <= band_y1)
                ++swamp_cells;
        }

        state.counters["path_len"] = static_cast<double>(path.size());
        state.counters["swamp_cells"] = static_cast<double>(swamp_cells);
        state.counters["total_cost"] = total_cost;
    }
}
BENCHMARK(BM_Weight_MWxTW)
    ->Args({1, 1})       // mw=0.1  tw=0.01  distance-cheap, terrain-ignored
    ->Args({1, 1000})    // mw=0.1  tw=10.0  distance-cheap, terrain-heavy
    ->Args({100, 1})     // mw=10.0 tw=0.01  distance-heavy, terrain-ignored
    ->Args({100, 1000})  // mw=10.0 tw=10.0  both heavy — balanced competition
    ->Args({10, 100})    // mw=1.0  tw=1.0   default
    ->ArgNames({"mw_x10", "tw_x100"})
    ->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
