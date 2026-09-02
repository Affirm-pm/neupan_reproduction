// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_motion_decorator/smooth_path_params.hpp"

namespace finenav::motion_decorator {

/**
 * @brief 无状态路径平滑 planner（map-less），参照 Nav2 SimpleSmoother 的迭代梯度下降平滑算法。
 *
 * 核心公式（对每个内部点 i，固定首尾不动）：
 *   y_i += data_w * (x_i - y_i) + smooth_w * (y_{i-1} + y_{i+1} - 2 * y_i)
 *
 * 其中：
 *   - x_i  = 原始路径点（锚定项，防止路径偏移过远）
 *   - y_i  = 当前平滑后的路径点
 *   - data_w   = 数据权重，越大越贴近原始路径
 *   - smooth_w = 平滑权重，越大曲线越光滑
 *
 * 特性：
 *   - 仅修改 poses 中的 position（x, y），不修改 z 和 orientation
 *   - 首尾点固定不动（保证起点终点不变）
 *   - 少于 3 个点时静默跳过（无法平滑）
 */
class SmoothPath {
public:
    using ConfigType = smooth_path::Params;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    SmoothPath();
    void configure(const ConfigType& cfg);
    void reset();

    using Context = finenav::core::PlanningContext<SmoothPath>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<SmoothPath>>;

    Result plan(const Context& ctx);

private:
    double data_w_{0.2};
    double smooth_w_{0.3};
    double tolerance_{1e-4};
    int    max_iterations_{1000};
};

}  // namespace finenav::motion_decorator
