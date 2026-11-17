// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/**
 * @file eval_scenario.cpp
 * @brief Route Planner 独立可视化评估 — 含局部传感器 replan 场景
 *
 * 四个场景:
 *   1 = 空地图对角线 (A* vs D* Lite 基准对比)
 *   2 = ★ 局部传感器多障碍物replan —
 *       机器人携带有限传感器范围, 边走边发现未知障碍物,
 *       D* Lite 局部修复 vs A* 全量重搜索 (时间对比)
 *   3 = 地形带权重扫描
 *   4 = ★ 随机未知障碍物持续探索
 *
 * @author Jinghui Wang (wangjinghui@sjtu.edu.cn)
 * @author IWIN-FINS Lab, Shanghai Jiao Tong University
 * @date 2026-07-21
 */

#include "eval_config.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "grid_map.hpp"
#include "finenav_route_planner/astar/astar_algorithm.hpp"
#include "finenav_route_planner/dstarlite/dstarlite_algorithm.hpp"

namespace fs = std::filesystem;

// ===== 公共类型 =====

struct PathResult {
    std::string algo;
    std::string scenario;
    std::string label;
    int         run = 0;
    std::vector<astar::Vec3> path;
    double      compute_ms   = 0.0;
    double      total_terrain_cost = 0.0;
    int         discovery_step = -1;  // -1 = N/A, ≥0 = discovery step
};

struct EvalRun {
    std::string              name;
    std::vector<PathResult>  paths;
};

// ===== 辅助函数 =====

static astar::GridMap buildEmptyGrid(int size) {
    return astar::GridMap(size, size, 0.0);
}

static astar::GridMap buildTerrainBandGrid(int size,
                                            int x0, int x1, int y0, int y1,
                                            double cost) {
    astar::GridMap grid(size, size, 0.0);
    for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
            grid.setCost(x, y, cost);
    return grid;
}

/// 在网格上放置一个障碍物方块
static void placeObstacleBlock(astar::GridMap& grid, int cx, int cy, int half) {
    for (int dy = -half; dy <= half; ++dy)
        for (int dx = -half; dx <= half; ++dx) {
            int x = cx + dx, y = cy + dy;
            if (x >= 0 && x < grid.getWidth() && y >= 0 && y < grid.getHeight())
                grid.setObstacle(x, y, true);
        }
}

static auto makeLambdas(const astar::GridMap& g) {
    return std::make_pair(
        [&g](const astar::Vec3& c) { return g.isWalkableCell(c.x, c.y); },
        [&g](const astar::Vec3& c) { return g.getCostCell(c.x, c.y); }
    );
}

// Like makeLambdas, but folds terrain_weight into the cost callback. The new
// AStarAlgorithm has no setTerrainWeight(), so the weight is applied here to
// keep the edge cost step * (movement_weight + terrain_weight * cost).
static auto makeLambdasWeighted(const astar::GridMap& g, double terrain_weight) {
    return std::make_pair(
        [&g](const astar::Vec3& c) { return g.isWalkableCell(c.x, c.y); },
        [&g, terrain_weight](const astar::Vec3& c) { return terrain_weight * g.getCostCell(c.x, c.y); }
    );
}

static astar::Heuristic toHeuristic(int h) {
    switch (h) {
        case 0:  return astar::Heuristic::Euclidean;
        case 1:  return astar::Heuristic::Diagonal;
        case 2:  return astar::Heuristic::Manhattan;
        case 3:  return astar::Heuristic::Zero;
        default: return astar::Heuristic::Euclidean;
    }
}

static astar::Connectivity toConn(int c) {
    return (c == 4) ? astar::Connectivity::N4 : astar::Connectivity::N8;
}

static double terrainCostSum(const std::vector<astar::Vec3>& path,
                              const astar::GridMap& grid) {
    double sum = 0.0;
    for (const auto& c : path)
        sum += grid.getCostCell(c.x, c.y);
    return sum;
}

static void ensureOutputDir() {
    std::error_code ec;
    fs::create_directories(cfg::kOutputDir, ec);
}

// ===== 高精度计时 =====

using Clock = std::chrono::high_resolution_clock;

static double elapsedMs(Clock::time_point t0, Clock::time_point t1) {
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

// ===== CSV 输出 =====

static void writePathsCSV(const std::vector<PathResult>& results) {
    ensureOutputDir();
    std::string p = std::string(cfg::kOutputDir) + "/paths.csv";
    std::ofstream f(p);
    if (!f) { std::cerr << "[ERROR] Cannot write " << p << "\n"; return; }
    f << "algo,scenario,run,step,x,y,label\n";
    for (const auto& r : results) {
        for (size_t i = 0; i < r.path.size(); ++i) {
            f << r.algo << "," << r.scenario << "," << r.run << ","
              << i << "," << r.path[i].x << "," << r.path[i].y << ","
              << r.label << "\n";
        }
    }
    std::cout << "[paths.csv] " << results.size() << " paths → " << p << "\n";
}

static void writeStatsCSV(const std::vector<PathResult>& results) {
    ensureOutputDir();
    std::string p = std::string(cfg::kOutputDir) + "/stats.csv";
    std::ofstream f(p);
    if (!f) { std::cerr << "[ERROR] Cannot write " << p << "\n"; return; }
    f << "algo,scenario,run,path_type,length,compute_ms,total_terrain_cost,discovery_step\n";
    for (const auto& r : results) {
        f << r.algo << "," << r.scenario << "," << r.run << ","
          << r.label << "," << r.path.size() << "," << r.compute_ms << ","
          << r.total_terrain_cost << "," << r.discovery_step << "\n";
    }
    std::cout << "[stats.csv] " << results.size() << " rows → " << p << "\n";
}

/// 将障碍物格坐标写入 _config.json（供 plot.py 可视化）
/// 调用方先 push 障碍物位置，再调用此函数
static std::vector<astar::Vec3> g_obstacle_cells;

static void writeConfigJSON() {
    ensureOutputDir();
    std::string p = std::string(cfg::kOutputDir) + "/_config.json";
    std::ofstream f(p);
    if (!f) return;
    f << "{\n"
      << "  \"grid_size\": " << cfg::kGridSize << ",\n"
      << "  \"sensor_range\": " << cfg::kSensorRange << ",\n"
      << "  \"band_x0\": " << cfg::kBandX0 << ",\n"
      << "  \"band_x1\": " << cfg::kBandX1 << ",\n"
      << "  \"band_y0\": " << cfg::kBandY0 << ",\n"
      << "  \"band_y1\": " << cfg::kBandY1 << ",\n"
      << "  \"band_cost\": " << cfg::kTerrainBandCost << ",\n"
      << "  \"obstacles\": [";
    for (size_t i = 0; i < g_obstacle_cells.size(); ++i) {
        if (i > 0) f << ",";
        f << "[" << g_obstacle_cells[i].x << "," << g_obstacle_cells[i].y << "]";
    }
    f << "]\n}\n";
    g_obstacle_cells.clear();
    std::cout << "[_config.json] → " << p << "\n";
}

// =========================================================================
// Scenario 1 — 空地图对角线 (A* vs D* Lite 基准对比)
// =========================================================================
static EvalRun runScenario1() {
    EvalRun run;
    run.name = "empty_diagonal";
    const int sz = cfg::kGridSize;
    astar::GridMap grid = buildEmptyGrid(sz);
    auto [iw, gc] = makeLambdas(grid);
    std::vector<PathResult> results;

    if constexpr (cfg::kRunAStar) {
        PathResult r;
        r.algo = "AStar"; r.scenario = "empty_diagonal"; r.label = "initial";
        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(toConn(cfg::kConnectivity));
        algo.setHeuristic(toHeuristic(cfg::kHeuristicAStar));
        algo.setGoalTolerance(cfg::kGoalTolerance);
        auto t0 = Clock::now();
        auto opt = algo.findPath({cfg::kStartX, cfg::kStartY, 0},
                                  {cfg::kGoalX,  cfg::kGoalY,  0});
        auto t1 = Clock::now();
        r.compute_ms = elapsedMs(t0, t1);
        if (opt) { r.path = *opt; r.total_terrain_cost = terrainCostSum(r.path, grid); }
        results.push_back(r);
    }

    if constexpr (cfg::kRunDStarLite) {
        PathResult r;
        r.algo = "DStarLite"; r.scenario = "empty_diagonal"; r.label = "initial";
        dstarlite::DStarLiteAlgorithm engine(sz, sz, iw, gc,
                                          toConn(cfg::kConnectivity),
                                          toHeuristic(cfg::kHeuristicDSL));
        engine.setMovementWeight(cfg::kMovementWeight);
        engine.setGreedyWeight(cfg::kGreedyWeight);
        engine.setTerrainWeight(cfg::kTerrainWeight);
        engine.initialize({cfg::kStartX, cfg::kStartY, 0},
                           {cfg::kGoalX,  cfg::kGoalY,  0});
        auto t0 = Clock::now();
        engine.computeShortestPath();
        auto t1 = Clock::now();
        r.compute_ms = elapsedMs(t0, t1);
        r.path = engine.extractPath();
        r.total_terrain_cost = terrainCostSum(r.path, grid);
        results.push_back(r);
    }

    run.paths = results;
    return run;
}

// =========================================================================
// Scenario 2 — ★ 局部传感器多障碍物 replan
//
// 机器人从 (0,0) 沿对角线前往 (99,99), 先验地图为空。
// 路径上秘密埋藏 3 个障碍物簇, 机器人每 kScanInterval 步用
// syncMapAround 扫描, 发现后 D* Lite 局部修复, A* 全量重搜索。
// 公平对比: 两算法使用相同的启发式 (Euclidean)。
// =========================================================================
static EvalRun runScenario2() {
    EvalRun run;
    run.name = "wall_replan";
    std::vector<PathResult> results;
    const auto cmp_heuristic = toHeuristic(cfg::kHeuristicAStar);
    const astar::Connectivity conn = toConn(cfg::kConnectivity);

    // 3 个障碍物簇分布在对角线附近
    const struct { int cx, cy, half; } secret_clusters[] = {
        {30, 30, 1}, {55, 56, 1}, {80, 78, 1},
    };
    constexpr int kNumClusters = 3;

    if constexpr (cfg::kRunDStarLite) {
        astar::GridMap grid = buildEmptyGrid(cfg::kGridSize);
        auto [iw, gc] = makeLambdas(grid);

        dstarlite::DStarLiteAlgorithm engine(cfg::kGridSize, cfg::kGridSize,
                                          iw, gc, conn, cmp_heuristic);
        engine.setMovementWeight(cfg::kMovementWeight);
        engine.setGreedyWeight(cfg::kGreedyWeight);
        engine.setTerrainWeight(cfg::kTerrainWeight);

        // 初始规划（基于空地图）
        auto t0 = Clock::now();
        engine.initialize({0, 0, 0}, {cfg::kGridSize-1, cfg::kGridSize-1, 0});
        engine.computeShortestPath();
        auto t1 = Clock::now();

        PathResult init_r;
        init_r.algo = "DStarLite"; init_r.scenario = "wall_replan";
        init_r.label = "initial";
        init_r.path = engine.extractPath();
        init_r.compute_ms = elapsedMs(t0, t1);
        init_r.total_terrain_cost = terrainCostSum(init_r.path, grid);
        results.push_back(init_r);

        if (!init_r.path.empty()) {
            // ★ 障碍物在机器人运动过程中动态出现（不是一开始全放好）
            //    出现时机：第 N 步时放置第 N 个簇
            //    放置的障碍物写入 GridMap（回调能读到），但 engine 不知情
            //    下一次 syncMapAround 扫描到才会发现
            const int appear_steps[] = {20, 45, 70};  // 每个簇出现的步数
            int next_appear_idx = 0;

            // 步进 + 局部传感器扫描
            std::vector<astar::Vec3> trajectory;
            trajectory.push_back(engine.start());

            int step_count = 0;
            int discovery_count = 0;
            double total_replan_ms = 0.0;
            int zero_cost_scans = 0;
            int total_scans = 0;

            while (step_count < cfg::kMaxSteps) {
                if (engine.start() == engine.goal()) break;

                // ── 动态出现障碍物 ──
                if (next_appear_idx < kNumClusters &&
                    step_count >= appear_steps[next_appear_idx]) {
                    int cx = secret_clusters[next_appear_idx].cx;
                    int cy = secret_clusters[next_appear_idx].cy;
                    int h  = secret_clusters[next_appear_idx].half;
                    placeObstacleBlock(grid, cx, cy, h);
                    for (int dy = -h; dy <= h; ++dy)
                        for (int dx = -h; dx <= h; ++dx)
                            g_obstacle_cells.push_back({cx + dx, cy + dy, 0});
                    std::cout << "[S2] Obstacle #" << (next_appear_idx + 1)
                              << " appeared @ step " << step_count
                              << " at (" << cx << "," << cy << ")\n";
                    next_appear_idx++;
                }

                if (!engine.step()) {
                    engine.syncMapAround(engine.start(), cfg::kSensorRange);
                    engine.replan(engine.start());
                    if (!engine.step()) break;
                }
                step_count++;
                trajectory.push_back(engine.start());

                if (step_count % cfg::kScanInterval == 0) {
                    engine.syncMapAround(engine.start(), cfg::kSensorRange);
                    total_scans++;
                    auto tr0 = Clock::now();
                    engine.replan(engine.start());
                    auto tr1 = Clock::now();
                    double dt = elapsedMs(tr0, tr1);
                    total_replan_ms += dt;
                    if (dt < 0.01) zero_cost_scans++;
                    else {
                        discovery_count++;
                        std::cout << "[S2] ★ Discovered @ step " << step_count
                                  << " robot=(" << engine.start().x
                                  << "," << engine.start().y << ")\n";
                    }
                }
            }

            std::cout << "[S2] D* Lite: " << discovery_count << " discoveries, "
                      << zero_cost_scans << "/" << total_scans
                      << " scans free. Total replan: " << total_replan_ms << " ms.\n";

            // 轨迹
            PathResult traj_r;
            traj_r.algo = "DStarLite"; traj_r.scenario = "wall_replan";
            traj_r.label = "trajectory";
            traj_r.path = trajectory;
            results.push_back(traj_r);

            // D* Lite 累计
            PathResult dsl_sum;
            dsl_sum.algo = "DStarLite"; dsl_sum.scenario = "wall_replan";
            dsl_sum.label = "cumulative_replan";
            dsl_sum.compute_ms = total_replan_ms;
            dsl_sum.discovery_step = discovery_count;
            results.push_back(dsl_sum);

            // ── A* 对比：每次发现后全量重搜索 ──
            if constexpr (cfg::kRunAStar) { if (discovery_count > 0) {
                double total_astar_ms = 0.0;
                // 在轨迹中点采样一次 A* 全量搜索耗时, 乘以发现次数估算
                int mid = std::min(step_count / 2,
                                   std::max(5, (int)trajectory.size() - 5));
                if (mid < (int)trajectory.size()) {
                    auto [iw2, gc2] = makeLambdasWeighted(grid, cfg::kTerrainWeight);
                    astar::AStarAlgorithm algo(iw2, gc2);
                    algo.setConnectivity(conn);
                    algo.setHeuristic(cmp_heuristic);
                    algo.setGoalTolerance(cfg::kGoalTolerance);
                    algo.setMovementWeight(cfg::kMovementWeight);
                    algo.setGreedyWeight(cfg::kGreedyWeight);

                    auto ta0 = Clock::now();
                    algo.findPath(trajectory[mid],
                                  {cfg::kGridSize-1, cfg::kGridSize-1, 0});
                    auto ta1 = Clock::now();
                    total_astar_ms = elapsedMs(ta0, ta1) * discovery_count;
                }

                PathResult astar_r;
                astar_r.algo = "AStar"; astar_r.scenario = "wall_replan";
                astar_r.label = "cumulative_est";
                astar_r.compute_ms = total_astar_ms;
                astar_r.discovery_step = discovery_count;
                results.push_back(astar_r);
            }}
        }
    }

    // A* 基准（空地图 — 与 D* Lite initial 公平对比搜同一张图）
    if constexpr (cfg::kRunAStar) {
        astar::GridMap grid_empty = buildEmptyGrid(cfg::kGridSize);
        auto [iw0, gc0] = makeLambdas(grid_empty);
        astar::AStarAlgorithm algo0(iw0, gc0);
        algo0.setConnectivity(conn);
        algo0.setHeuristic(cmp_heuristic);
        algo0.setGoalTolerance(cfg::kGoalTolerance);
        auto ta0 = Clock::now();
        algo0.findPath({0, 0, 0}, {cfg::kGridSize-1, cfg::kGridSize-1, 0});
        auto ta1 = Clock::now();
        PathResult astar_empty;
        astar_empty.algo = "AStar"; astar_empty.scenario = "wall_replan";
        astar_empty.label = "empty_map_baseline";
        astar_empty.compute_ms = elapsedMs(ta0, ta1);
        results.push_back(astar_empty);
    }

    // A* 基线（含所有障碍物的完整地图）
    if constexpr (cfg::kRunAStar) {
        astar::GridMap grid_full = buildEmptyGrid(cfg::kGridSize);
        for (int i = 0; i < kNumClusters; ++i)
            placeObstacleBlock(grid_full, secret_clusters[i].cx,
                               secret_clusters[i].cy, secret_clusters[i].half);
        auto [iw, gc] = makeLambdas(grid_full);
        astar::AStarAlgorithm algo(iw, gc);
        algo.setConnectivity(conn);
        algo.setHeuristic(cmp_heuristic);
        algo.setGoalTolerance(cfg::kGoalTolerance);
        auto ta0 = Clock::now();
        auto opt = algo.findPath({0, 0, 0}, {cfg::kGridSize-1, cfg::kGridSize-1, 0});
        auto ta1 = Clock::now();
        PathResult astar_init;
        astar_init.algo = "AStar"; astar_init.scenario = "wall_replan";
        astar_init.label = "initial_w_obstacles";
        astar_init.compute_ms = elapsedMs(ta0, ta1);
        if (opt) {
            astar_init.path = *opt;
            astar_init.total_terrain_cost = terrainCostSum(astar_init.path, grid_full);
        }
        results.push_back(astar_init);
    }

    run.paths = results;
    return run;
}

// =========================================================================
// Scenario 3 — 地形带权重扫描
// =========================================================================
static EvalRun runScenario3() {
    EvalRun run;
    run.name = "terrain_sweep";
    const int sz = cfg::kGridSize;
    astar::GridMap grid = buildTerrainBandGrid(
        sz, cfg::kBandX0, cfg::kBandX1,
        cfg::kBandY0, cfg::kBandY1, cfg::kTerrainBandCost);
    auto [iw, gc] = makeLambdas(grid);
    std::vector<PathResult> results;

    for (int wi = 0; wi < cfg::kWeightSweepCount; ++wi) {
        double tw = cfg::kWeightSweep[wi];

        if constexpr (cfg::kRunDStarLite) {
            PathResult r;
            r.algo = "DStarLite"; r.scenario = "terrain_sweep"; r.run = wi;
            char buf[32];
            std::snprintf(buf, sizeof(buf), "tw=%.2f", tw);
            r.label = buf;
            dstarlite::DStarLiteAlgorithm engine(sz, sz, iw, gc,
                                              toConn(cfg::kConnectivity),
                                              toHeuristic(cfg::kHeuristicDSL));
            engine.setMovementWeight(cfg::kMovementWeight);
            engine.setGreedyWeight(cfg::kGreedyWeight);
            engine.setTerrainWeight(tw);
            engine.initialize({cfg::kStartX, 50, 0}, {cfg::kGoalX, 50, 0});
            auto t0 = Clock::now();
            engine.computeShortestPath();
            auto t1 = Clock::now();
            r.compute_ms = elapsedMs(t0, t1);
            r.path = engine.extractPath();
            r.total_terrain_cost = terrainCostSum(r.path, grid);
            results.push_back(r);
        }

        if constexpr (cfg::kRunAStar) {
            PathResult r;
            r.algo = "AStar"; r.scenario = "terrain_sweep"; r.run = wi;
            char buf[32];
            std::snprintf(buf, sizeof(buf), "tw=%.2f", tw);
            r.label = buf;
            auto [iw_w, gc_w] = makeLambdasWeighted(grid, tw);
            astar::AStarAlgorithm algo(iw_w, gc_w);
            algo.setConnectivity(toConn(cfg::kConnectivity));
            algo.setHeuristic(toHeuristic(cfg::kHeuristicAStar));
            algo.setGoalTolerance(cfg::kGoalTolerance);
            algo.setMovementWeight(cfg::kMovementWeight);
            algo.setGreedyWeight(cfg::kGreedyWeight);
            auto t0 = Clock::now();
            auto opt = algo.findPath({cfg::kStartX, 50, 0}, {cfg::kGoalX, 50, 0});
            auto t1 = Clock::now();
            r.compute_ms = elapsedMs(t0, t1);
            if (opt) { r.path = *opt; r.total_terrain_cost = terrainCostSum(r.path, grid); }
            results.push_back(r);
        }
    }

    run.paths = results;
    return run;
}

// =========================================================================
// Scenario 4 — ★ 随机未知障碍物持续探索
// =========================================================================
static EvalRun runScenario4() {
    EvalRun run;
    run.name = "random_discovery";
    const int sz = cfg::kGridSize;
    std::vector<PathResult> results;
    const auto cmp_heuristic = toHeuristic(cfg::kHeuristicAStar);
    const astar::Connectivity conn = toConn(cfg::kConnectivity);

    std::mt19937 rng(cfg::kRandomSeed);
    std::uniform_int_distribution<int> dist_offset(-15, 15);
    astar::GridMap prior_grid(sz, sz, 0.0);
    std::vector<astar::Vec3> secret_obstacles;

    int placed = 0;
    while (placed < cfg::kRandomObstacleCount) {
        double frac = (double)(placed + 1) / (cfg::kRandomObstacleCount + 1);
        int base_x = (int)(frac * (sz - 1));
        int base_y = (int)(frac * (sz - 1));
        int ox = std::clamp(base_x + dist_offset(rng), 5, sz - 6);
        int oy = std::clamp(base_y + dist_offset(rng), 5, sz - 6);
        for (int dy = 0; dy < 2 && placed < cfg::kRandomObstacleCount; ++dy)
            for (int dx = 0; dx < 2 && placed < cfg::kRandomObstacleCount; ++dx) {
                int bx = ox + dx, by = oy + dy;
                if (bx < 3 && by < 3) continue;
                if (bx >= sz-3 && by >= sz-3) continue;
                secret_obstacles.push_back({bx, by, 0});
                placed++;
            }
    }
    if ((int)secret_obstacles.size() > cfg::kRandomObstacleCount)
        secret_obstacles.resize(cfg::kRandomObstacleCount);
    std::cout << "[S4] Placed " << secret_obstacles.size()
              << " obstacles along diagonal\n";

    if constexpr (cfg::kRunDStarLite) {
        astar::GridMap grid = prior_grid;
        auto [iw, gc] = makeLambdas(grid);
        dstarlite::DStarLiteAlgorithm engine(sz, sz, iw, gc, conn, cmp_heuristic);
        engine.setMovementWeight(cfg::kMovementWeight);
        engine.setGreedyWeight(cfg::kGreedyWeight);
        engine.setTerrainWeight(cfg::kTerrainWeight);

        auto t0 = Clock::now();
        engine.initialize({0, 0, 0}, {sz-1, sz-1, 0});
        engine.computeShortestPath();
        auto t1 = Clock::now();

        PathResult init_r;
        init_r.algo = "DStarLite"; init_r.scenario = "random_discovery";
        init_r.label = "initial";
        init_r.path = engine.extractPath();
        init_r.compute_ms = elapsedMs(t0, t1);
        results.push_back(init_r);

        // ★ 障碍物动态出现：每个障碍物在机器人接近时（约距 sensor_range 格）才出现
        //    先按距离排序
        std::sort(secret_obstacles.begin(), secret_obstacles.end(),
            [](const astar::Vec3& a, const astar::Vec3& b) {
                return (a.x + a.y) < (b.x + b.y);  // Manhattan distance from (0,0)
            });
        size_t next_obs_idx = 0;
        int obs_appeared = 0;

        std::vector<astar::Vec3> trajectory;
        trajectory.push_back(engine.start());
        int step_count = 0;
        int discoveries = 0;
        double total_replan_ms = 0.0;
        int zero_cost_scans = 0;
        int total_scans = 0;

        while (step_count < cfg::kMaxSteps) {
            if (engine.start() == engine.goal()) break;

            // ── 动态出现障碍物：只在机器人前方且接近时放置 ──
            while (next_obs_idx < secret_obstacles.size()) {
                const auto& obs = secret_obstacles[next_obs_idx];
                // 只放前方的障碍物（防止出现在身后）
                if (obs.x + obs.y <= engine.start().x + engine.start().y) {
                    next_obs_idx++;  // 已经过了，跳过（但仍然写入 grid 供可视化）
                    grid.setObstacle(obs.x, obs.y, true);
                    g_obstacle_cells.push_back(obs);
                    obs_appeared++;
                    continue;
                }
                int dist = std::abs(engine.start().x - obs.x)
                         + std::abs(engine.start().y - obs.y);
                if (dist > cfg::kSensorRange + 5) break;  // 还不够近
                grid.setObstacle(obs.x, obs.y, true);
                g_obstacle_cells.push_back(obs);
                next_obs_idx++;
                obs_appeared++;
            }

            if (!engine.step()) {
                engine.syncMapAround(engine.start(), cfg::kSensorRange);
                engine.replan(engine.start());
                if (!engine.step()) break;
            }
            step_count++;
            trajectory.push_back(engine.start());

            if (step_count % cfg::kScanInterval == 0) {
                engine.syncMapAround(engine.start(), cfg::kSensorRange);
                total_scans++;
                auto tr0 = Clock::now();
                engine.replan(engine.start());
                auto tr1 = Clock::now();
                double dt = elapsedMs(tr0, tr1);
                total_replan_ms += dt;
                if (dt < 0.01) zero_cost_scans++;
                else discoveries++;
            }
        }

        PathResult traj_r;
        traj_r.algo = "DStarLite"; traj_r.scenario = "random_discovery";
        traj_r.label = "trajectory";
        traj_r.path = trajectory;
        results.push_back(traj_r);

        PathResult stat_r;
        stat_r.algo = "DStarLite"; stat_r.scenario = "random_discovery";
        stat_r.label = "cumulative_replan";
        stat_r.compute_ms = total_replan_ms;
        stat_r.discovery_step = discoveries;
        results.push_back(stat_r);

        std::cout << "[S4] D* Lite: " << discoveries << " discoveries, "
                  << zero_cost_scans << "/" << total_scans
                  << " scans free. Total replan: " << total_replan_ms << " ms.\n";

        if constexpr (cfg::kRunAStar) {
            int mid = std::min(step_count / 2,
                               std::max(5, (int)trajectory.size() - 5));
            if (mid < (int)trajectory.size() && discoveries > 0) {
                auto [iw2, gc2] = makeLambdasWeighted(grid, cfg::kTerrainWeight);
                astar::AStarAlgorithm algo(iw2, gc2);
                algo.setConnectivity(conn);
                algo.setHeuristic(cmp_heuristic);
                algo.setGoalTolerance(cfg::kGoalTolerance);
                algo.setMovementWeight(cfg::kMovementWeight);
                algo.setGreedyWeight(cfg::kGreedyWeight);
                auto ta0 = Clock::now();
                algo.findPath(trajectory[mid], {sz-1, sz-1, 0});
                auto ta1 = Clock::now();
                double astar_est = elapsedMs(ta0, ta1) * discoveries;

                PathResult astar_r;
                astar_r.algo = "AStar"; astar_r.scenario = "random_discovery";
                astar_r.label = "cumulative_est";
                astar_r.compute_ms = astar_est;
                astar_r.discovery_step = discoveries;
                results.push_back(astar_r);
            }
        }
    }

    run.paths = results;
    return run;
}

// ===== main =====

struct StageTimer {
    const char* name;
    Clock::time_point t0;
    explicit StageTimer(const char* n) : name(n), t0(Clock::now()) {}
    ~StageTimer() {
        auto t1 = Clock::now();
        std::cout << "[" << name << "] done in " << elapsedMs(t0, t1) << " ms\n";
    }
};

static void printResultTable(const std::vector<PathResult>& results) {
    if (results.empty()) return;
    const int w_algo = 10, w_label = 22, w_len = 8, w_time = 12, w_disc = 8;
    auto sep = [&](const char* l, const char* m, const char* r) {
        std::cout << l;
        for (int i = 0; i < w_algo + w_label + w_len + w_time + w_disc + 19; ++i)
            std::cout << "─";
        std::cout << r << "\n";
    };
    sep("┌", "┬", "┐");
    printf("│ %-*s │ %-*s │ %*s │ %*s │ %*s │\n",
           w_algo, "Algorithm", w_label, "Stage",
           w_len, "Length", w_time, "Time(ms)", w_disc, "#Disc");
    sep("├", "┼", "┤");
    for (const auto& r : results) {
        printf("│ %-*s │ %-*s │ %*d │ %*.3f │ %*d │\n",
               w_algo, r.algo.c_str(),
               w_label, r.label.c_str(),
               w_len,   (int)r.path.size(),
               w_time,  r.compute_ms,
               w_disc,  r.discovery_step);
    }
    sep("└", "┴", "┘");

    std::cout << "\n";
}

static void finishScenario(const EvalRun& run) {
    writePathsCSV(run.paths);
    writeStatsCSV(run.paths);
    writeConfigJSON();
    std::cout << "\n── " << run.name << " ──\n";
    printResultTable(run.paths);
    std::string cmd = std::string("python3 ") + EVAL_DIR + "/plot.py "
                    + std::string(cfg::kOutputDir);
    std::cout << "\n[plot] " << std::flush;
    int ret = std::system(cmd.c_str());
    if (ret != 0)
        std::cerr << "[plot] Warning: plot.py exited with code " << ret << "\n";
    std::error_code ec;
    fs::remove(std::string(cfg::kOutputDir) + "/paths.csv", ec);
    fs::remove(std::string(cfg::kOutputDir) + "/stats.csv", ec);
    fs::remove(std::string(cfg::kOutputDir) + "/_config.json", ec);
    std::cout << "\n";
}

int main() {
    std::cout << "═══════════════════════════════════════════\n";
    std::cout << " Route Planner — Standalone Eval\n";
    std::cout << " Scenario: " << (cfg::kScenario == 0 ? "ALL (1+2+3+4)" :
                                  cfg::kScenario == 1 ? "1 — empty_diagonal" :
                                  cfg::kScenario == 2 ? "2 — wall_replan (local sensor)" :
                                  cfg::kScenario == 3 ? "3 — terrain_sweep" :
                                  "4 — random_discovery") << "\n";
    std::cout << " Grid:     " << cfg::kGridSize << "×" << cfg::kGridSize << "\n";
    std::cout << " Sensor:   range=" << cfg::kSensorRange
              << " cells, scan every " << cfg::kScanInterval << " steps\n";
    std::cout << " A*:       " << (cfg::kRunAStar     ? "ON" : "OFF") << "\n";
    std::cout << " D* Lite:  " << (cfg::kRunDStarLite ? "ON" : "OFF") << "\n";
    std::cout << " Output:   " << cfg::kOutputDir << "\n";
    std::cout << "═══════════════════════════════════════════\n\n";

    if constexpr (cfg::kScenario == 0) {
        { StageTimer st("Scenario 1"); finishScenario(runScenario1()); }
        { StageTimer st("Scenario 2"); finishScenario(runScenario2()); }
        { StageTimer st("Scenario 3"); finishScenario(runScenario3()); }
        { StageTimer st("Scenario 4"); finishScenario(runScenario4()); }
        std::cout << "\n[Done] PNGs in " << cfg::kOutputDir << "\n";
    } else {
        EvalRun run;
        switch (cfg::kScenario) {
            case 1: run = runScenario1(); break;
            case 2: run = runScenario2(); break;
            case 3: run = runScenario3(); break;
            case 4: run = runScenario4(); break;
            default:
                std::cerr << "[ERROR] Unknown kScenario\n"; return 1;
        }
        finishScenario(run);
    }
    return 0;
}
