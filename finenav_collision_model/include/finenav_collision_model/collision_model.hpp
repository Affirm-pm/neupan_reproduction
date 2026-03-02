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

    // TODO: 这里将碰撞检测的作用局限在了一种特定的 map: Gridmap<float>
    // OBB (Oriented Bounding Box) 本质上只是一个纯粹的几何学概念（长、宽、高、朝向）。它不需要什么要知道世界上存在一种东西叫“地图”？
    // 建议改为，让用户给我们一个函数对象，让它告诉我们什么是碰撞
    // template <typename CollisionRule>
    // bool checkCollision(const OBB& obb, const Pose& pose, CollisionRule rule) {
    //      ...
    //     for (...) {
    //         if (rule(x, y, z)) return true;
    //     }
    // }
    //
    virtual bool checkCollision(const Pose& pose, const GridMap<float>& map) const = 0;
};

} // namespace finenav
