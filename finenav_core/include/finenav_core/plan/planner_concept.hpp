// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <concepts>
#include <optional>
#include <utility>

#include "finenav_core/plan/trajectory.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_interfaces/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace finenav::core {

/**
 * @brief PrePlanContext — context passed to pre-plan callbacks
 */

struct PrePlanContext {
    const std::string&           active_planner_id;
    RobotState&                  robot_state;
    geometry_msgs::msg::Pose&    goal_pose;
    finenav_interfaces::msg::Trajectory& ref_traj;

/**
 * @brief PlanningContext — context passed to planner.plan() calls.
 * @tparam PlannerT Planner type
 */
template <typename PlannerT> // TODO: Compile guard with PlannerConcept?
struct PlanningContext {
    RobotState robot_state;  ///< Current robot state
    geometry_msgs::msg::Pose      goal_pose;
    InputTrajectoryT<PlannerT>    ref_traj;
/**
 * @brief PostPlanContext — context passed to post-plan callbacks.
 */
struct PostPlanContext {
    const std::string&                    active_planner_id;
    const bool                            is_success;
    finenav_interfaces::msg::Trajectory&  result_traj;
};


/**
 * @brief PlannerConcept
 */
template <typename T>
concept PlannerConcept = // TODO: 改名为 Planner
    std::default_initializable<T> &&
    requires {
        typename T::ConfigType;
        typename T::IMapView;
        typename InputTrajectoryT<T>;
        typename OutputTrajectoryT<T>;
    } &&
    requires(T planner,
             const typename T::ConfigType& config,
             const PlanningContext<T>& ctx,
             const typename T::IMapView& map_view) {

        // --- Basic configuration ---
        { planner.configure(config) } -> std::same_as<void>;

        // --- Episode boundary: called by framework when a new goal arrives ---
        { planner.reset() } -> std::same_as<void>;

        // --- Core planning interface ---
        { planner.plan(ctx, map_view) } -> std::same_as<std::optional<OutputTrajectoryT<T>>>;
    };

}  // namespace finenav::core
