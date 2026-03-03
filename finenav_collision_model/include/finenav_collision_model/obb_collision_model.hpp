// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_collision_model/collision_model.hpp"
#include <stdexcept>
#include <vector>

namespace finenav {

/**
 * @brief OBBCollisionModel 的参数
 * @param step 表面采样步长（单位：m），控制碰撞检测的空间精度，由用户决定，default=0.1m
 */
struct OBBCollisionPolicy {
    double step = 0.1; ///< 表面采样步长（m）
};

/**
 * @brief OBB (Oriented Bounding Box) 碰撞模型
 *
 * @details
 * 碰撞判断完全由调用方通过 CollisionRule 提供，因此可适配任意地图类型
 * 碰撞检测分为两阶段以支持各种场景:
 *   - precompute(pose)：          在机器人位姿改变时调用一次，缓存世界系下的所有采样点。
 *   - checkCollision(rule)：      高频接口，直接遍历缓存点，零几何变换开销，需先调用 precompute。
 *   - checkCollision(pose, rule)：低频接口，内部自动 precompute，适用于普通调用。
 */
class OBBCollisionModel {
public:
    using Policy = OBBCollisionPolicy;

    /**
     * @brief 构造函数
     * @param size      OBB 在三个轴方向的尺寸 (L, W, H)
     * @param local_pose OBB 在机器人本体坐标系下的位姿（含旋转和偏移），默认为单位变换
     * @param policy    采样策略参数
     */
    explicit OBBCollisionModel(const Vector3D& size,
                               const Pose& local_pose = Pose::Identity(),
                               const Policy& policy   = Policy{});

    /**
     * @brief 预计算并缓存世界坐标系下的所有表面采样点
     * @details 只需在机器人位姿改变时调用一次。
     *          调用后可多次调用 checkCollision(rule) 而无需重复几何变换。
     * @param pose 机器人在世界坐标系下的位姿
     */
    void precompute(const Pose& pose);

    /**
     * @brief 高频碰撞检测（使用缓存采样点）
     * @details 调用前须先调用 precompute(pose)，适用于 MPPI 等需要极低延迟的场景。
     * @note 此接口为 const。低频重载因内部调用 precompute（修改缓存）而为非 const，
     *       须在对象变为 const 之前由外部提前调用 precompute(pose) 填充缓存。
     * @tparam CR 满足 CollisionRule concept 的可调用对象
     * @param rule 碰撞判断函数，由调用方提供
     * @return 任意一个采样点触发 rule 返回 true 时，整体返回 true
     */
    template <CollisionRule CR>
    bool checkCollision(CR&& rule) const;

    /**
     * @brief 低频碰撞检测（自动 precompute）
     * @details 内部自动调用 precompute(pose) 后执行检测，无需手动管理缓存，适用于普通场景。
     * @tparam CR 满足 CollisionRule concept 的可调用对象
     * @param pose 机器人在世界坐标系下的位姿
     * @param rule 碰撞判断函数
     * @return 如果发生碰撞返回 true
     */
    template <CollisionRule CR>
    bool checkCollision(const Pose& pose, CR&& rule);

private:
    Vector3D size_;       ///< OBB 尺寸
    Vector3D min_;        ///< OBB 自身坐标系下的最小边界
    Vector3D max_;        ///< OBB 自身坐标系下的最大边界
    Pose     local_pose_; ///< OBB 在机器人本体坐标系下的位姿（旋转 + 偏移）
    Policy   policy_;     ///< 采样策略参数

    std::vector<Position3D> cached_points_; ///< 缓存的世界系采样点（由 precompute 填充）
};

template <CollisionRule CR>
bool OBBCollisionModel::checkCollision(CR&& rule) const {
    if (cached_points_.empty()) {
        throw std::logic_error(
            "OBBCollisionModel::checkCollision called before precompute");
    }
    for (const auto& pt : cached_points_) {
        if (rule(pt)) {
            return true;
        }
    }
    return false;
}

template <CollisionRule CR>
bool OBBCollisionModel::checkCollision(const Pose& pose, CR&& rule) {
    precompute(pose);
    return checkCollision(std::forward<CR>(rule));
}

} // namespace finenav
