// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_core/plan/detail/plan_registry.hpp"

#include "finenav_util/common/log_macros.hpp"

namespace finenav::core::detail {

// Lazy function-local static defined in finenav_core.so — the single shared
// planner table across every plugin DSO (see FINENAV_DEFINE_REGISTRY_SINGLETON).
FINENAV_DEFINE_REGISTRY_SINGLETON(PlanRegistry)

void PlanRegistry::registerPlannerImpl(std::string name, Factory factory,
                                       common::detail::ConfigLoader loader, std::string type_name) {
    bool inserted = insertOrKeep(name, PlanEntry{
        std::move(factory), std::move(loader), std::move(type_name)});
    if (!inserted) {
        FINENAV_LOG_WARN(rclcpp::get_logger("PlanRegistry"),
            "Planner '{}' already registered — keeping the FIRST registrant "
            "(renaming one of the duplicate parameter namespaces resolves this).",
            name);
    } else {
        FINENAV_LOG_DEBUG(rclcpp::get_logger("PlanRegistry"),
            "Planner '{}' registered.", name);
    }
}

std::shared_ptr<IPipelineNode> PlanRegistry::createLayer(
    rclcpp::Node::SharedPtr node,
    const std::string& algo_name, const std::string& layer_name,
    ExecutionPolicyVariant policy, RobotStateProvider* state)
{
    auto entry = lookup(algo_name);
    if (!entry) {
        FINENAV_LOG_ERROR(rclcpp::get_logger("PlanRegistry"),
            "createLayer: planner '{}' is not registered. "
            "Available planners: [{}]",
            algo_name, common::detail::joinNames(names()));
        return nullptr;
    }
    return entry->factory(node, layer_name, std::move(policy), state);
}

std::shared_ptr<common::IParamPollable> PlanRegistry::loadPlannerParams(
    const std::string& algo_name, rclcpp::Node::SharedPtr node, void* instance)
{
    auto entry = lookup(algo_name);
    if (!entry) return nullptr;
    return entry->loader(node, instance);
}

std::vector<std::string> PlanRegistry::plannerNames() const {
    return names();
}

std::optional<std::string> PlanRegistry::findPlannerNameByType(
    const std::string& type_name) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [name, entry] : entries_) {
        if (entry.type == type_name) return name;
    }
    return std::nullopt;
}

}  // namespace finenav::core::detail
