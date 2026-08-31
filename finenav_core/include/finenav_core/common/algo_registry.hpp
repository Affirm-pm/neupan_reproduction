// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/map/i_map_resource.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/i_control_layer.hpp"
#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_core/common/param_pollable_keeper.hpp"

namespace finenav::core {

template <Planner PlannerT> class ControlLayer;
template <Map MapT> class MapServer;

/**
 * @brief AlgoRegistry — the single runtime reflection table for algorithms.
 * @details
 * Every algorithm .so self-registers here at static-init
 * One entry carries BOTH:
 *   1. a type-erased factory  → (node, name, policy, state) → IControlLayer /
 *      (node, name, state) → IMapResource
 *   2. a type-erased config loader → (node, void* instance) → IParamPollable
 *
 * @note Duplicate names keep the FIRST registrant and emit a warning.
 * @note The singleton must be a non-inline, lazy function-local static DEFINED in
 * finenav_core.so (see algo_registry.cpp) so that all DSOs share ONE table.
 */
class AlgoRegistry {
public:
    using PlannerFactory = std::function<std::shared_ptr<IControlLayer>(
        rclcpp::Node::SharedPtr, const std::string&, ExecutionPolicyVariant, RobotStateProvider*)>;
    using MapFactory = std::function<std::shared_ptr<IMapResource>(
        rclcpp::Node::SharedPtr, const std::string&, RobotStateProvider*)>;
    using ConfigLoader = std::function<std::shared_ptr<IParamPollable>(
        rclcpp::Node::SharedPtr, void*)>;

    static AlgoRegistry& instance();

    /** @brief Self registration
     *  @param name Registry key
     *  @param loader Config loader of the algo
     */
    template <Planner PlannerT>
    void registerPlanner(std::string name, ConfigLoader loader) {
        registerPlannerImpl(
            name,
            [name](rclcpp::Node::SharedPtr node, const std::string& layer_name,
                   ExecutionPolicyVariant policy, RobotStateProvider* state)
                -> std::shared_ptr<IControlLayer> {
                return std::make_shared<ControlLayer<PlannerT>>(
                    node, name, layer_name, std::move(policy), state);
            },
            std::move(loader));
    }

    /** @brief Self registration
     *  @param name Registry key
     *  @param loader Config loader of the algo
     */
    template <Map MapT>
    void registerMap(std::string name, ConfigLoader loader) {
        registerMapImpl(
            name,
            [name](rclcpp::Node::SharedPtr node, const std::string& server_name,
                   RobotStateProvider* state) -> std::shared_ptr<IMapResource> {
                return std::make_shared<MapServer<MapT>>(node, name, server_name, state);
            },
            std::move(loader));
    }

    // ── By-name construction (loud failure on unknown names) ──────────────

    /// Create a ControlLayer for the planner @p algo_name.  Returns nullptr +
    /// ERROR log if the name is not registered.
    std::shared_ptr<IControlLayer> createLayer(
        rclcpp::Node::SharedPtr node,
        const std::string& algo_name, const std::string& layer_name,
        ExecutionPolicyVariant policy, RobotStateProvider* state);

    /// Create a MapServer for the map @p algo_name.  Returns nullptr + ERROR
    /// log if the name is not registered.
    std::shared_ptr<IMapResource> createMap(
        rclcpp::Node::SharedPtr node,
        const std::string& algo_name, const std::string& server_name,
        RobotStateProvider* state);

    // ── By-name config loading (name-addressable reflection) ──────────────

    /// Look up the config loader registered under @p algo_name for a planner.
    /// Returns nullptr if the name is unknown.  @p instance is cast back to
    /// the concrete type inside the loader closure.
    std::shared_ptr<IParamPollable> loadPlannerParams(
        const std::string& algo_name, rclcpp::Node::SharedPtr node, void* instance);
    /// Look up the config loader registered under @p algo_name for a map.
    std::shared_ptr<IParamPollable> loadMapParams(
        const std::string& algo_name, rclcpp::Node::SharedPtr node, void* instance);

    // ── Introspection ─────────────────────────────────────────────────────

    std::vector<std::string> plannerNames() const;
    std::vector<std::string> mapNames() const;

private:
    void registerPlannerImpl(std::string name, PlannerFactory factory, ConfigLoader loader);
    void registerMapImpl(std::string name, MapFactory factory, ConfigLoader loader);

    // Sorted name lists.  Caller must hold mutex_.
    std::vector<std::string> plannerNamesLocked() const;
    std::vector<std::string> mapNamesLocked() const;

    // ", "-joined list for error messages.
    static std::string joinNames(const std::vector<std::string>& names);

    // Name → {factory, config loader}.  Insertion keeps the FIRST registrant.
    std::unordered_map<std::string, std::pair<PlannerFactory, ConfigLoader>> planner_entries_;
    std::unordered_map<std::string, std::pair<MapFactory, ConfigLoader>> map_entries_;
    mutable std::mutex mutex_;
};

namespace detail {

// Self-registrars
// It should be instantiated as `inline static`. C++17 guarantees a single
// entity across all TUs, so a header included by multiple .cpps registers exactly once

template <typename T>
struct PlannerSelfRegistrar {
    explicit PlannerSelfRegistrar(std::string name, AlgoRegistry::ConfigLoader loader) {
        AlgoRegistry::instance().registerPlanner<T>(std::move(name), std::move(loader));
    }
};

template <typename T>
struct MapSelfRegistrar {
    explicit MapSelfRegistrar(std::string name, AlgoRegistry::ConfigLoader loader) {
        AlgoRegistry::instance().registerMap<T>(std::move(name), std::move(loader));
    }
};

}  // namespace detail

}  // namespace finenav::core
