// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// ─────────────────────────────────────────────────────────────────────────────
//  Route Planner — 独立评测参数配置（无 ROS 依赖）
//
//  使用方法：
//    1. 修改下面的参数
//    2. colcon build --packages-select finenav_route_planner
//    3. ./build/finenav_route_planner/eval/route_planner_eval
//
//  场景说明：
//    1 = 空地图对角线（A* vs D* Lite 基准对比）
//    2 = ★ 局部传感器 replan（机器人step移动+sensor范围发现障碍物）
//    3 = 地形代价权重扫描
//    4 = ★ 随机未知障碍物持续探索（体现增量 vs 全量性能差距）
//    0 = 依次运行全部场景
//
//  输出：
//    /tmp/dstarlite_eval/paths.csv     — 路径坐标（含机器人轨迹）
//    /tmp/dstarlite_eval/stats.csv     — 耗时、长度、代价、扩展节点数
//    /tmp/dstarlite_eval/_config.json  — 参数快照（plot.py 自动读取）
// ─────────────────────────────────────────────────────────────────────────────

namespace cfg {

// ── 1. 场景选择 ──────────────────────────────────────────────────────────────
constexpr int kScenario = 0;

// ── 2. 算法开关 ──────────────────────────────────────────────────────────────
constexpr bool kRunAStar     = true;
constexpr bool kRunDStarLite = true;

// ── 3. 网格与连通性（所有场景共用）───────────────────────────────────────────
constexpr int kGridSize       = 100;
constexpr int kConnectivity   = 8;
constexpr int kHeuristicAStar = 0;   // Euclidean
constexpr int kHeuristicDSL   = 2;   // Manhattan

// ── 4. 代价权重（所有场景共用）───────────────────────────────────────────────
constexpr double kMovementWeight = 1.0;
constexpr double kGreedyWeight   = 1.0;
constexpr double kTerrainWeight  = 1.0;
constexpr double kGoalTolerance  = 0.0;

// ── 5. 起点 / 终点（场景 1-4 共用）───────────────────────────────────────────
constexpr int kStartX = 0,   kStartY = 0;
constexpr int kGoalX  = 99,  kGoalY  = 99;

// ── 6. ★ 传感器模型（场景 2、4 使用）─────────────────────────────────────────
//   sensor_range:    机器人传感器探测半径（Chebyshev 距离，栅格数）
//   scan_interval:   每移动多少步扫描一次周围变化
//   max_steps:       场景最大步数（防止死循环）
//
//   真实对应：
//     10 格 × 0.05m/格 = 0.5m 探测范围（模拟局部传感器视野）
constexpr int kSensorRange  = 12;  // 需 > closure 宽度（3格）+ 到达距离余量
constexpr int kScanInterval = 2;
constexpr int kMaxSteps     = 2000;

// ── 7. 场景二：墙 + 缺口 + 封口（局部传感器 replan）─────────────────────────
//
//   地图布局：
//        y=99 ┌──────────────┐
//             │              │
//        y=70 │  ██████████  │  ← 上半段墙
//        y=55 │  ██封口██  │  ← 秘密封口（机器人不知道）
//        y=45 │  ██封口██  │
//        y=30 │  ██████████  │  ← 下半段墙
//             │              │
//        y=0  └──────────────┘
//             x=50
//
//   流程：
//     1. 先验地图：只有两段墙+缺口 y∈(30,70)，机器人据此初始化
//     2. 外部秘密添加封口 y∈(45,55)
//     3. 机器人沿路径 step 前进，每 kScanInterval 步调用 syncMapAround
//     4. 当传感器范围触达封口 → 发现变化 → 增量 replan
//     5. 对比 D* Lite 增量扩展节点数 vs A* 全量重搜索节点数
//
constexpr int kWallX     = 50;   // 墙的 x 坐标
constexpr int kGapY0     = 30;   // 缺口下边界（下半段墙顶）
constexpr int kGapY1     = 70;   // 缺口上边界（上半段墙底）
constexpr int kClosureY0 = 45;   // 封口下边界
constexpr int kClosureY1 = 55;   // 封口上边界
// 场景二的起点/终点（水平穿过缺口）
constexpr int kS2StartX = 0,  kS2StartY = 50;
constexpr int kS2GoalX  = 99, kS2GoalY  = 50;

// ── 8. 场景三：地形代价带权重扫描 ────────────────────────────────────────────
constexpr double kTerrainBandCost   = 10.0;
constexpr int    kBandX0 = 35, kBandX1 = 45;
constexpr int    kBandY0 = 10, kBandY1 = 89;

inline constexpr double kWeightSweep[] = {
    0.01, 0.05, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0, 2.0, 5.0
};
inline constexpr int kWeightSweepCount =
    static_cast<int>(sizeof(kWeightSweep) / sizeof(kWeightSweep[0]));

// ── 9. ★ 场景四：随机未知障碍物持续探索 ─────────────────────────────────────
//
//   在空地图上随机散布 N 个障碍物块，机器人不知道它们的位置。
//   机器人沿对角线 (0,0)→(99,99) 前进，边走边用传感器发现障碍物，
//   每发现一个就增量 replan。
//
//   对比：
//     - D* Lite：累计增量扩展节点数
//     - A*：每次全量重搜索的累计节点数
//     两者差距越大，越体现增量算法的价值
//
constexpr int    kRandomObstacleCount = 30;   // 随机障碍物数量
constexpr double kObstacleDensity     = 0.03; // 障碍物密度（% of grid cells）
constexpr int    kRandomSeed          = 42;   // 随机种子（固定以复现结果）

// ── 10. 输出目录 ─────────────────────────────────────────────────────────────
constexpr const char* kOutputDir = "/tmp/dstarlite_eval";

}  // namespace cfg
