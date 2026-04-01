// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <concepts>
#include <optional>
#include <utility>

#include "finenav_msgs/msg/map_chunk.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_core/plan/data_packet.hpp"

namespace finenav {

/**
 * @brief MapConcept
 */
template <typename T>
concept MapConcept =
    std::default_initializable<T> &&
    requires(T map, const typename T::ConfigType& config) {
        { map.configure(config) } -> std::same_as<bool>; // 配置地图参数
    };

/**
 * @brief PlanningContext (规划上下文)
 * @details 包含算法在“这一帧”规划所需的所有不可变数据。
 */
template <typename PlannerT>
struct PlanningContext {
    geometry_msgs::msg::Pose start_pose;
    geometry_msgs::msg::Pose goal_pose;
    InputDataT<PlannerT> ref_traj; // TODO: 验证空轨迹的逻辑
};

/**
 * @brief PlannerConcept TODO：是否考虑改名为 Motion Generator
 * @details
 * Planner 被定义为一个访问地图试图并生成轨迹的算法模块，P: (State, Reference, Map) -> Trajectory
 * * 在外部视角下，核心特性：
 * 1. 特质自省： 暴露它的类型特质，以区分是否需要参考轨迹
 * 2. 不持有地图，只接受地图试图 MapView
 */
template <typename T>
concept PlannerConcept = std::default_initializable<T> &&
    requires {
        typename T::ConfigType;
        typename T::IMapView;
        typename InputDataT<T>;
        typename OutputDataT<T>;
    } &&
    requires(T planner, const typename T::ConfigType& config, const PlanningContext<T>& ctx, const typename T::IMapView& map_view) {

        // --- 基本配置 ---
        { planner.configure(config) } -> std::same_as<void>;

        // --- 规划接口 ---
        { planner.plan(ctx, map_view) } -> std::same_as<std::optional<OutputDataT<T>>>;
};


}
