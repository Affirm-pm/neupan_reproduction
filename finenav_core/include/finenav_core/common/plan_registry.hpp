// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <typeinfo>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/common/named_registry.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/i_control_layer.hpp"
#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/robot_state/robot_state.hpp"

namespace finenav::core {

template <Planner PlannerT> class ControlLayer;  // full definition in control_layer.hpp

/// One planner's registry row: name → { factory, config loader, type }.
struct PlanEntry {
    using Factory = std::function<std::shared_ptr<IControlLayer>(
        rclcpp::Node::SharedPtr, const std::string&, ExecutionPolicyVariant, RobotStateProvider*)>;

    Factory      factory;  ///< (node, layer_name, policy, state) → IControlLayer
    ConfigLoader loader;   ///< (node, void* planner) → IParamPollable
    std::string  type;     ///< typeid(PlannerT).name()
};

/**
 * @brief PlanRegistry — the runtime reflection table for PLANNERS.
 *
 * A planner .so self-registers here (via FINENAV_REGISTER_PLANNER): name →
 * { factory, config loader, type }.  The engine looks the factory up by name to
 * build a ControlLayer, and reverse-resolves type→name when assembling a
 * MapView.
 */
class PlanRegistry : public RegistryTable<PlanEntry> {
public:
    using Factory = PlanEntry::Factory;
    using Entry   = PlanEntry;

    static PlanRegistry& instance();

    /// Self-registration: record the planner factory + config loader under @p name.
    template <Planner PlannerT>
    void registerPlanner(std::string name, ConfigLoader loader);

    /// Build a ControlLayer for the planner registered under @p algo_name.
    /// Returns nullptr + ERROR log on unknown name.
    std::shared_ptr<IControlLayer> createLayer(
        rclcpp::Node::SharedPtr node,
        const std::string& algo_name, const std::string& layer_name,
        ExecutionPolicyVariant policy, RobotStateProvider* state);

    /// Look up the config loader registered under @p algo_name (nullptr if unknown).
    std::shared_ptr<IParamPollable> loadPlannerParams(
        const std::string& algo_name, rclcpp::Node::SharedPtr node, void* instance);

    std::vector<std::string> plannerNames() const;

    /// Resolve the registered NAME for a concrete planner type (typeid(PlannerT).name()).
    std::optional<std::string> findPlannerNameByType(const std::string& type_name) const;

private:
    void registerPlannerImpl(std::string name, Factory factory,
                             ConfigLoader loader, std::string type_name);
};

template <Planner PlannerT>
void PlanRegistry::registerPlanner(std::string name, ConfigLoader loader) {
    registerPlannerImpl(
        name,
        [name](rclcpp::Node::SharedPtr node, const std::string& layer_name,
               ExecutionPolicyVariant policy, RobotStateProvider* state)
            -> std::shared_ptr<IControlLayer> {
            return std::make_shared<ControlLayer<PlannerT>>(
                node, name, layer_name, std::move(policy), state);
        },
        std::move(loader),
        typeid(PlannerT).name());
}

namespace detail {

/// Self-registrar for a planner; instantiated as `inline static` in the macro.
template <typename T>
struct PlannerSelfRegistrar {
    explicit PlannerSelfRegistrar(std::string name, ConfigLoader loader) {
        PlanRegistry::instance().registerPlanner<T>(std::move(name), std::move(loader));
    }
};

}  // namespace detail
}  // namespace finenav::core
