// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_collision_model/obb_collision_model.hpp"
#include <stdexcept>

namespace finenav {

OBBCollisionModel::OBBCollisionModel(const Vector3D& size, const Pose& local_pose, const Policy& policy)
    : size_(size), local_pose_(local_pose), policy_(policy) {
    if (policy_.step <= 0.0) {
        throw std::invalid_argument(
            "OBBCollisionModel: policy.step must be greater than 0");
    }
    min_ = -size_ / 2.0;
    max_ =  size_ / 2.0;
}

void OBBCollisionModel::precompute(const Pose& pose) {
    cached_points_.clear();

    const double step = policy_.step;

    // 预先合并两次变换：OBB 本体系 → 机器人本体系 → 世界系
    // 避免在每个采样点重复计算 pose * local_pose_
    const Pose body_to_world = pose * local_pose_;

    // 遍历 OBB 的 6 个面（每个轴各 min/max 两个面）进行表面采样
    // 轴旋转技巧：对于固定轴 axis，u = (axis+1)%3，v = (axis+2)%3
    for (int axis = 0; axis < 3; ++axis) {
        const int u_axis = (axis + 1) % 3;
        const int v_axis = (axis + 2) % 3;

        for (const double side : {min_[axis], max_[axis]}) {
            for (double u = min_[u_axis]; u <= max_[u_axis]; u += step) {
                for (double v = min_[v_axis]; v <= max_[v_axis]; v += step) {
                    Position3D local_pt;
                    local_pt[axis]   = side;
                    local_pt[u_axis] = u;
                    local_pt[v_axis] = v;
                    cached_points_.push_back(body_to_world * local_pt);
                }
                // 补全 v 方向最大边界（浮点步进可能漏掉 max_v）
                Position3D edge_v;
                edge_v[axis]   = side;
                edge_v[u_axis] = u;
                edge_v[v_axis] = max_[v_axis];
                cached_points_.push_back(body_to_world * edge_v);
            }
            // 补全 u 方向最大边界列（浮点步进可能漏掉 max_u）
            for (double v = min_[v_axis]; v <= max_[v_axis]; v += step) {
                Position3D edge_u;
                edge_u[axis]   = side;
                edge_u[u_axis] = max_[u_axis];
                edge_u[v_axis] = v;
                cached_points_.push_back(body_to_world * edge_u);
            }
            // 补全 (max_u, max_v) 角点（edge_v 和 edge_u 均未覆盖此交叉点）
            Position3D corner;
            corner[axis]   = side;
            corner[u_axis] = max_[u_axis];
            corner[v_axis] = max_[v_axis];
            cached_points_.push_back(body_to_world * corner);
        }
    }
}

} // namespace finenav
