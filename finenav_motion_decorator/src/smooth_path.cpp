// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_motion_decorator/smooth_path.hpp"

#include <cmath>
#include <vector>

#include "finenav_core/common/algo_register_macros.hpp"

namespace finenav::motion_decorator {

SmoothPath::SmoothPath() = default;

void SmoothPath::configure(const ConfigType& cfg) {
    data_w_          = cfg.data_weight;
    smooth_w_        = cfg.smooth_weight;
    tolerance_       = cfg.tolerance;
    max_iterations_  = cfg.max_iterations;
}

void SmoothPath::reset() {}

SmoothPath::Result SmoothPath::plan(const Context& ctx) {
    using Trajectory = finenav::core::Trajectory<OutputProfile>;

    Trajectory result;
    result.poses = ctx.ref_traj.poses;

    const std::size_t n = result.poses.size();

    // 少于 3 个点无法平滑，静默跳过。
    if (n < 3) {
        return result;
    }

    // 保存原始路径作为锚定参考（仅 x, y）。
    struct Point2D { double x; double y; };
    std::vector<Point2D> original(n);
    for (std::size_t i = 0; i < n; ++i) {
        original[i].x = result.poses[i].position.x;
        original[i].y = result.poses[i].position.y;
    }

    // 迭代拉普拉斯平滑（首尾固定）。
    for (int iter = 0; iter < max_iterations_; ++iter) {
        double change = 0.0;

        for (std::size_t i = 1; i < n - 1; ++i) {
            auto& p   = result.poses[i].position;
            auto& pm1 = result.poses[i - 1].position;
            auto& pp1 = result.poses[i + 1].position;

            const double x_i = original[i].x;
            const double y_i = original[i].y;

            // 拉普拉斯平滑 + 数据锚定。
            const double new_x = p.x
                + data_w_   * (x_i - p.x)
                + smooth_w_ * (pm1.x + pp1.x - 2.0 * p.x);
            const double new_y = p.y
                + data_w_   * (y_i - p.y)
                + smooth_w_ * (pm1.y + pp1.y - 2.0 * p.y);

            const double dx = new_x - p.x;
            const double dy = new_y - p.y;
            change += std::fabs(dx) + std::fabs(dy);

            p.x = new_x;
            p.y = new_y;
        }

        // 收敛判断。
        if (change < tolerance_) {
            break;
        }
    }

    return result;
}

}  // namespace finenav::motion_decorator

FINENAV_REGISTER_PLANNER(finenav::motion_decorator::SmoothPath, smooth_path)
