// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_collision_model/obb_collision_model.hpp"
#include <cmath>

namespace finenav {

//构造函数,初始化OBB的尺寸和边界，定义机器人坐标系下OBB的最小边界和最大边界，policy存储碰撞检测策略参数
//local_pose存储OBB在机器人坐标系下的位姿，通过传入的size参数计算出OBB在机器人坐标系下的最小边界和最大边界
OBBCollisionModel::OBBCollisionModel(const Vector3D& size, const Pose& local_pose, const Policy& policy)
    : size_(size), local_pose_(local_pose), policy_(policy) {
    min_ = -size_ / 2.0;
    max_ = size_ / 2.0;
}

//检测函数
bool OBBCollisionModel::checkCollision(const Pose& pose, const GridMap<float>& map) const {
    double res = map.getResolution();

    // 防止分辨率为 0 导致死循环
    if (res <= 0.0) {
        return false;
    }

    // 遍历 OBB 的 6 个面进行采样
    // 采样步长设为地图分辨率，目前选择将面上的所有点都视作采样点
    double step = res;

    // 定义 3 个轴的方向索引
    for (int axis = 0; axis < 3; ++axis) {
        int u_axis = (axis + 1) % 3;
        int v_axis = (axis + 2) % 3;

        // 每个轴有两个面 (min 和 max)
        for (double side : {min_[axis], max_[axis]}) {
            for (double u = min_[u_axis]; u <= max_[u_axis]; u += step) {
                for (double v = min_[v_axis]; v <= max_[v_axis]; v += step) {
                    Position3D local_pt;
                    local_pt[axis] = side;
                    local_pt[u_axis] = u;
                    local_pt[v_axis] = v;

                    // 先经过 OBB 局部位姿（本体系内旋转/偏移），再变换到世界坐标系
                    Position3D world_pt = pose * local_pose_ * local_pt;

                    if (isPointCollide(world_pt, map)) {
                        return true;
                    }
                }
                // 确保包含 v 方向最大边界点（浮点步进可能漏掉 max_v）
                Position3D local_pt_edge;
                local_pt_edge[axis] = side;
                local_pt_edge[u_axis] = u;
                local_pt_edge[v_axis] = max_[v_axis];
                if (isPointCollide(pose * local_pose_ * local_pt_edge, map)) return true;
            }
            // 确保包含 u 方向最大边界线（浮点步进可能漏掉 max_u 整列）
            for (double v = min_[v_axis]; v <= max_[v_axis]; v += step) {
                Position3D local_pt_edge;
                local_pt_edge[axis] = side;
                local_pt_edge[u_axis] = max_[u_axis];
                local_pt_edge[v_axis] = v;
                if (isPointCollide(pose * local_pose_ * local_pt_edge, map)) return true;
            }
        }
    }

    return false;
}

bool OBBCollisionModel::isPointCollide(const Position3D& pos, const GridMap<float>& map) const {
    if (!map.isInside(pos)) {
        return policy_.out_of_bounds_as_collision; // 可配置：地图外是否视为碰撞
    }

    try {
        float val = map.atPosition(pos);
        // 大于threshold且不是NaN的值视为碰撞
        if (!std::isnan(val) && val > policy_.occupancy_threshold) {
            return true;
        }
    } catch (const std::out_of_range&) {
        return false;
    }

    return false;
}

} // namespace finenav
