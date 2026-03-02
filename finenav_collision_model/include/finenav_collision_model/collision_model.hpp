// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/math_types.hpp"
#include "grid_map.hpp"

namespace finenav {

/**
 * @brief 碰撞模型基类
 * @details 碰撞模型应当是无状态的算法，提供碰撞检测接口
 */
class CollisionModel {
public:
    virtual ~CollisionModel() = default;

    /**
     * @brief 检查碰撞
     * @param pose 机器人在世界坐标系下的位姿
     * @param map 栅格地图
     * @return 如果发生碰撞返回true，否则返回false
     */
    virtual bool checkCollision(const Pose& pose, const GridMap<float>& map) const = 0;
};

} // namespace finenav
