// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <concepts>

#include "finenav_core/plan/planner_concept.hpp"

namespace finenav::core {

/**
 * @brief WorldView — the world as THIS robot sees it.
 *
 * A WorldView bridges a concrete Map (the world: objective data, independent of
 * who is looking at it) to the IWorldView a Planner consumes (that world with
 * the robot's footprint, pose and task folded in).  The planner acts on the
 * robot's perspective, never on the raw environment.
 *
 * It is built ONCE per layer and lives as long as the layer, so it MAY hold
 * arbitrary internal state (incremental terrain fields, inflation buffers,
 * subscription caches, ...).
 *
 * Two capabilities are MANDATORY:
 *
 *   - `configure(const ConfigType&)` — the parameter channel, exactly as for a
 *     Planner and a Map.  Declared with a generate_parameter_library YAML;
 *     called once at startup and again on every `ros2 param set`.
 *
 *   - `prepare(const PlanningContext<PlannerT>&)` — called ONCE PER plan(),
 *     inside the map read lock and BEFORE plan().  This is the ONLY place heavy
 *     derivation belongs: robot-body inflation, terrain analysis, cost-field
 *     construction — anything that reads the maps or needs the robot pose/goal.
 *     Whatever plan() calls (isWalkable, isCollision, getCost, ...) must be
 *     cheap and read only what prepare() left behind.
 *
 * Everything else is OPTIONAL — declare these and the framework injects them,
 * omit them and nothing breaks:
 *
 *   - `setLogger(rclcpp::Logger)`
 *   - `setVisualizer(finenav::viz::FineNavVisualizer&)`
 *
 * The constructor convention is
 *   `ViewT(rclcpp::Node::SharedPtr node, const Map1&, const Map2&, ...)`
 * — the layer's private sub-node first, then one `const Map&` per map type
 * declared at registration.
 *
 * Registration (one line, in the view's .cpp) enforces this concept:
 * @code
 *   FINENAV_REGISTER_WORLDVIEW(my_world_view, MyWorldView, my_world_view,
 *                              MyPlanner, MyMap);
 * @endcode
 */
template <typename T, typename PlannerT>
concept WorldView =
    requires(T view, const typename T::ConfigType& config) {
        typename T::ConfigType;
        { view.configure(config) } -> std::same_as<void>;
    } &&
    requires(T view, const PlanningContext<PlannerT>& ctx) {
        { view.prepare(ctx) } -> std::same_as<void>;
    };

}  // namespace finenav::core
