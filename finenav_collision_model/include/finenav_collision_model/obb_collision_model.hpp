// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_collision_model/collision_model.hpp"

namespace finenav {

/**
 * @brief OBBCollisionModel 的碰撞检测策略参数
 * @param occupancy_threshold      栅格占用判定阈值，栅格值大于此值时认为碰撞，默认为 0.0
 * @param out_of_bounds_as_collision 地图外的点是否视为碰撞，默认为 false（视为自由空间）
 */
struct OBBCollisionPolicy {
    float occupancy_threshold       = 0.0f;  ///< 栅格占用判定阈值，栅格值大于此值时认为碰撞
    bool  out_of_bounds_as_collision = false; ///< 地图外的点是否视为碰撞
};

/**
 * @brief OBB (Oriented Bounding Box) 碰撞模型
 * @details 描述一个包围盒，并提供基于表面采样点的碰撞检测
 */
// TODO: 建议不要用继承机制，至少现在不要
// 原因：在用户的视角里，他在编译期就能确定创建一个哪一个具体的 CollisionModel 对象，而不需要对它进行动态管理
// 这会让未来的其他碰撞检测工具僵化，比如我们之前提到的神奇的 RC-ESDF 方案

class OBBCollisionModel : public CollisionModel {
    public:
        using Policy = OBBCollisionPolicy;

        /**
         * @brief 构造函数
         * @param size OBB 在三个轴方向的尺寸 (L, W, H)
         * @param local_pose OBB 在机器人本体坐标系下的位姿（含旋转和偏移），默认为单位变换
         * @param policy 碰撞检测策略参数
         */
        explicit OBBCollisionModel(const Vector3D& size,
                                const Pose& local_pose = Pose::Identity(),
                                const Policy& policy = Policy{});

        /**
         * @brief 检查碰撞
         * @param pose 机器人在世界坐标系下的位姿
         * @param map 栅格地图
         * @return 如果发生碰撞返回true，否则返回false
         */
        // TODO: 性能相关，碰撞检测机制分为两步：(1) 找到世界坐标系下的所有检测点； (2) 遍历所有点进行检测
        // (1) 只有在机器人移动的时候才需要计算一次
        // 而 (2) 需要进行高频检测 【e.g. 调用一次 MPPI 规划时，规划器在此时的机器人位置和地图下，产生数千条轨迹，每条轨迹都要高频访问地图进行检测】
        // 目前 (1) 和 （2）完全耦合，导致每次调用都需要变换一次box，然后步进计算 point
        // 应当把两步分开，并且在内部维护一个缓存，存储当前 box 对应的所有 point
        bool checkCollision(const Pose& pose, const GridMap<float>& map) const override;

    private:
        /**
         * @brief 检查某个点是否与地图碰撞 (内部工具)
         * @param pos 世界坐标系下的点
         * @param map 栅格地图
         * @return 如果点所在栅格被占用返回true
         */
        bool isPointCollide(const Position3D& pos, const GridMap<float>& map) const;

        Vector3D size_;       ///< OBB 尺寸
        Vector3D min_;        ///< OBB 自身坐标系下的最小边界
        Vector3D max_;        ///< OBB 自身坐标系下的最大边界
        Pose     local_pose_; ///< OBB 在机器人本体坐标系下的位姿（旋转 + 偏移）
        Policy   policy_;     ///< 碰撞检测策略参数

        // TODO： box表面采样的步长应该是自己维护的属性，不应该直接向 map.getResolution() 索取
        // 原因：它代表了“用户希望以多高的精度进行检测”，而不应该由地图来反向决定，我们把这个值的合理性交给用户


};

} // namespace finenav
