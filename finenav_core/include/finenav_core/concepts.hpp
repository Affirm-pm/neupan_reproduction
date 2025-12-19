// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <concepts>
#include <optional>
#include <utility>

#include "math_types.hpp"
#include "finenav_msgs/msg/map_chunk.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_util/planner_traits.hpp"

namespace finenav {

/**
 * @brief MapConcept
 * @details
 * Map 被定义为一个具有视窗机制的空间数据容器，M: Postion3D -> DataType
 * * 在外部视角下，核心特性：
 * 1. 读写访问： 提供对特定空间位置的数据访问 (access) 和更新 (update) 接口
 * 2. 视窗机制: 仅允许操作以当前中心的矩形范围内的有效数据，视窗外的数据被视为失效（视窗可以被定义为无穷大）
 * 3. 数据融合: 区别于一般的容器，地图的更新操作通常涉及数据融合，提供自定义融合策略的接口
 *
 */
template <typename T>
concept MapConcept =
    std::default_initializable<T> &&
    std::movable<T> &&
    requires(T map, const Position3D& pos, const typename T::ConfigType& config, const typename T::DataType& data) {
        // --- 基本配置 ---
        typename T::ConfigType;
        typename T::DataType;
        { map.configure(config) } -> std::same_as<bool>; // 配置地图参数

        // --- 读写操作 ---
        { std::as_const(map).access(pos) } -> std::convertible_to<typename T::DataType>;
        { map.integrate(pos, data)} -> std::same_as<bool>; // 数据更新

        // --- 视窗操作 ---
        { std::as_const(map).isInside(pos) } -> std::same_as<bool>; // 检查是否在当前视窗内
        { std::as_const(map).getWindowBounds() } -> std::convertible_to<Region3D>; // 获取当前视窗范围
        { std::as_const(map).getWindowCenter() } -> std::convertible_to<Position3D>; // 获取当前视窗中心
        // map.shift_window(pos, cb); // TODO：之后再整，不要用回调了，直接用把 MapChunk 拿出来
};


/**
 * @brief PlanningContext (规划上下文)
 * @details 包含算法在“这一帧”规划所需的所有不可变数据。
 */
struct PlanningContext {
    geometry_msgs::msg::Pose start_pose;
    geometry_msgs::msg::Pose goal_pose;
    std::optional<finenav_msgs::msg::Trajectory> reference_path;  // 可选的参考路径
};

/**
 * @brief PlannerConcept TODO：是否考虑改名为 Motion Generator
 * @details
 * Planner 被定义为一个访问地图试图并生成轨迹的算法模块，P: (State, Reference, Map) -> Trajectory
 * * 在外部视角下，核心特性：
 * 1. 特质自省： 暴露它的类型特质，以区分是否需要参考轨迹
 * 2. 不持有地图，只接受地图试图 MapView
 */
template <typename T,  typename MapView>
concept PlannerConcept = std::default_initializable<T> &&
    requires(T planner, const MapView& map, const typename T::ConfigType& config, const PlanningContext& ctx, finenav_msgs::msg::Trajectory& output_traj) {
        typename T::ConfigType;

        { planner.configure(config) } -> std::same_as<void>;

        { planner.plan(map, ctx, output_traj) } -> std::same_as<bool>;
};

/**
 * @brief LayerConcept
 * @details
*/
template <typename T>
concept LayerTraits = requires {
    typename T::Map;      // 必须包含 MapType
    typename T::Planners;   // 必须包含 PlannerSet
    typename T::MapViews;    // 必须包含 MapViews
}; // TODO: 可能还要补充 PlannerSet 和 FilterSet 的类型检查，之后再说



}
