// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <concepts>
#include <optional>

#include "geometry_msgs/msg/pose.hpp"
#include "finenav_core/plan/trajectory.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_interfaces/msg/trajectory.hpp"

namespace finenav::core {

/**
 * @brief PrePlanContext — context passed to pre-plan callbacks
 */

struct PrePlanContext {
    const std::string&           active_planner_id;  ///< ID of the planner that is about to run (const)
    RobotState&                  robot_state;        ///< Current robot state
    geometry_msgs::msg::Pose&    goal_pose;          ///< Goal pose for the current planning episode
    finenav_interfaces::msg::Trajectory& ref_traj;   ///< Reference trajectory for the current planning episode
};

/**
 * @brief PlanningContext — context passed to planner.plan() calls.
 * @tparam PlannerT Planner type
 */
template <typename PlannerT>
struct PlanningContext {
    RobotState                    robot_state;      ///< Current robot state
    geometry_msgs::msg::Pose      goal_pose;        ///< Goal pose for the current planning episode
    InputTrajectoryT<PlannerT>    ref_traj;         ///< Reference trajectory for the current planning episode
};

/**
 * @brief PostPlanContext — context passed to post-plan callbacks.
 */
struct PostPlanContext {
    const std::string&                    active_planner_id;  ///< ID of the planner that just ran (const)
    const bool                            is_success;         ///< Whether the planning is success or not (const)
    finenav_interfaces::msg::Trajectory&  result_traj;        ///< Reference trajectory for the current planning episode
};


/**
 * @brief HasMapView — does the planner declare an IMapView (i.e. read a map)?
 *
 * A planner that reads a map declares `using IMapView = ...;` and its plan()
 * takes a map view.  A map-less planner (e.g. a pure trajectory decorator)
 * declares NO IMapView and its plan() takes only the context.  This structural
 * trait lets the framework dispatch between the two signatures.
 */
template <typename T>
concept HasMapView = requires { typename T::IMapView; };

/**
 * @brief Planner — the fundamental contract for motion generation algorithms.
 *
 * A Planner maps a reference trajectory to an output trajectory.  Whether it
 * reads a map is expressed by the plan() signature itself:
 *   - reads a map   → declares IMapView, plan(ctx, map_view)
 *   - map-less      → declares no IMapView, plan(ctx)
 */
template <typename T>
concept Planner =
    std::default_initializable<T> &&
    requires {
        typename T::ConfigType;
        typename InputTrajectoryT<T>;
        typename OutputTrajectoryT<T>;
    } &&
    requires(T planner,
             const typename T::ConfigType& config,
             const PlanningContext<T>& ctx) {

        // --- Basic configuration ---
        { planner.configure(config) } -> std::same_as<void>;

        // --- Episode boundary: called by framework when a new goal arrives ---
        { planner.reset() } -> std::same_as<void>;
    } &&
    (requires(T planner,
              const PlanningContext<T>& ctx,
              const typename T::IMapView& map_view) {
        // --- Core planning interface (map-reading planner) ---
        { planner.plan(ctx, map_view) } -> std::same_as<std::optional<OutputTrajectoryT<T>>>;
    } ||
    requires(T planner, const PlanningContext<T>& ctx) {
        // --- Core planning interface (map-less planner) ---
        { planner.plan(ctx) } -> std::same_as<std::optional<OutputTrajectoryT<T>>>;
    });

}  // namespace finenav::core
