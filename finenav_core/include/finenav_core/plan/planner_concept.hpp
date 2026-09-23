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
 * @brief Why there is no pre-plan / post-plan callback.
 *
 * Earlier revisions exposed `set_on_pre_plan` / `set_on_post_plan` on the
 * pipeline node.  They were removed before 1.0, and the reasoning is worth
 * keeping because the same request will come back:
 *
 *   - The pre-plan callback could not do the job it existed for.  Its
 *     `ref_traj` was a read-only view of the *incoming ROS message*, while the
 *     planner reads a different object (`cached_ref_`), so a rewrite in the
 *     callback would not have reached the planner even if the field had been
 *     mutable.  What the old demo built on it — pose-dependent seek, smooth and
 *     trim of the reference — belongs to an upstream decorator stage.
 *
 *   - The post-plan callback returned a `bool` that OVERRODE the plan outcome.
 *     A callback returning `true` unconditionally turned a failed plan into a
 *     default-constructed trajectory reported as success.  Discarding or
 *     clamping a result belongs to `navigator::IChassisCommand::validate()`.
 *
 *   - Visualisation, the other historical use, is served by injecting a
 *     `finenav::viz::FineNavVisualizer` into the algorithm.
 *
 * Anything added to this concept is a permanent compatibility promise.  If a
 * need reappears, give it a named concept of its own with a signature that
 * cannot be misused, rather than reopening a general-purpose callback slot.
 */

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
