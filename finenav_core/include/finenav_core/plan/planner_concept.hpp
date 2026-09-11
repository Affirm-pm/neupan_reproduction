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
    const std::string&                     active_planner_id;  ///< ID of the planner that is about to run (const) // TODO: 要删掉
    const RobotState&                      robot_state;        ///< Current robot state (read-only view)
    const finenav_interfaces::msg::Trajectory& ref_traj;       ///< Reference trajectory (read-only view)
};

/**
 * @brief PlanningContext — context passed to planner.plan() calls.
 *
 * The goal of an episode is carried as the last pose of the reference
 * trajectory (`ref_traj.poses.back()`), not as a separate field.
 *
 * Both fields are **read-only views** into framework-owned storage.  A planner
 * that needs to mutate or retain the reference must copy it itself — the
 * framework never hands ownership of the input to the planner.
 *
 * @tparam PlannerT Planner type
 */
template <typename PlannerT>
struct PlanningContext {
    const RobotState&                    robot_state;  ///< Current robot state (read-only view)
    const InputTrajectoryT<PlannerT>&    ref_traj;     ///< Reference trajectory (read-only view)
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
 * @brief HasWorldView — does the planner declare an IWorldView (i.e. read a map)?
 *
 * A planner that reads a map declares `using IWorldView = ...;` and its plan()
 * takes a map view.  A map-less planner (e.g. a pure trajectory decorator)
 * declares NO IWorldView and its plan() takes only the context.  This structural
 * trait lets the framework dispatch between the two signatures.
 */
template <typename T>
concept HasWorldView = requires { typename T::IWorldView; };

/**
 * @brief Planner — the fundamental contract for motion generation algorithms.
 *
 * A Planner maps a reference trajectory to an output trajectory.  Whether it
 * reads a map is expressed by the plan() signature itself:
 *   - reads a map   → declares IWorldView, plan(ctx, map_view)
 *   - map-less      → declares no IWorldView, plan(ctx)
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
              const typename T::IWorldView& map_view) {
        // --- Core planning interface (map-reading planner) ---
        { planner.plan(ctx, map_view) } -> std::same_as<std::optional<OutputTrajectoryT<T>>>;
    } ||
    requires(T planner, const PlanningContext<T>& ctx) {
        // --- Core planning interface (map-less planner) ---
        { planner.plan(ctx) } -> std::same_as<std::optional<OutputTrajectoryT<T>>>;
    });

}  // namespace finenav::core
