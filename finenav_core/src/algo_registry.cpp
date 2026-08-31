// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "../include/finenav_core/common/algo_registry.hpp"

#include <algorithm>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::core {

// ══════════════════════════════════════════════════════════════════════════════
//  Singleton
// ══════════════════════════════════════════════════════════════════════════════
//  Lazy function-local static defined in finenav_core.so — this is
//  what makes it a single shared table across every algorithm DSO.
AlgoRegistry& AlgoRegistry::instance() {
    static AlgoRegistry registry;
    return registry;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Registration
// ══════════════════════════════════════════════════════════════════════════════

void AlgoRegistry::registerPlannerImpl(std::string name, PlannerFactory factory, ConfigLoader loader) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto [it, inserted] = planner_entries_.emplace(
        name, std::make_pair(std::move(factory), std::move(loader)));
    if (!inserted) {
        FINENAV_LOG_WARN(rclcpp::get_logger("AlgoRegistry"),
            "Planner '{}' already registered — keeping the FIRST registrant "
            "(renaming one of the duplicate parameter namespaces resolves this).",
            name);
    } else {
        FINENAV_LOG_DEBUG(rclcpp::get_logger("AlgoRegistry"),
            "Planner '{}' registered.", name);
    }
}

void AlgoRegistry::registerMapImpl(std::string name, MapFactory factory, ConfigLoader loader) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto [it, inserted] = map_entries_.emplace(
        name, std::make_pair(std::move(factory), std::move(loader)));
    if (!inserted) {
        FINENAV_LOG_WARN(rclcpp::get_logger("AlgoRegistry"),
            "Map '{}' already registered — keeping the FIRST registrant "
            "(renaming one of the duplicate parameter namespaces resolves this).",
            name);
    } else {
        FINENAV_LOG_DEBUG(rclcpp::get_logger("AlgoRegistry"),
            "Map '{}' registered.", name);
    }
}

// ══════════════════════════════════════════════════════════════════════════════
//  By-name construction
// ══════════════════════════════════════════════════════════════════════════════

std::shared_ptr<IControlLayer> AlgoRegistry::createLayer(
    rclcpp::Node::SharedPtr node,
    const std::string& algo_name, const std::string& layer_name,
    ExecutionPolicyVariant policy, RobotStateProvider* state)
{
    PlannerFactory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = planner_entries_.find(algo_name);
        if (it == planner_entries_.end()) {
            FINENAV_LOG_ERROR(rclcpp::get_logger("AlgoRegistry"),
                "createLayer: planner '{}' is not registered. "
                "Available planners: [{}]",
                algo_name, joinNames(plannerNamesLocked()));
            return nullptr;
        }
        factory = it->second.first;
    }
    return factory(node, layer_name, std::move(policy), state);
}

std::shared_ptr<IMapResource> AlgoRegistry::createMap(
    rclcpp::Node::SharedPtr node,
    const std::string& algo_name, const std::string& server_name,
    RobotStateProvider* state)
{
    MapFactory factory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = map_entries_.find(algo_name);
        if (it == map_entries_.end()) {
            FINENAV_LOG_ERROR(rclcpp::get_logger("AlgoRegistry"),
                "createMap: map '{}' is not registered. "
                "Available maps: [{}]",
                algo_name, joinNames(mapNamesLocked()));
            return nullptr;
        }
        factory = it->second.first;
    }
    return factory(node, server_name, state);
}

// ══════════════════════════════════════════════════════════════════════════════
//  By-name config loading
// ══════════════════════════════════════════════════════════════════════════════

std::shared_ptr<IParamPollable> AlgoRegistry::loadPlannerParams(
    const std::string& algo_name, rclcpp::Node::SharedPtr node, void* instance)
{
    ConfigLoader loader;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = planner_entries_.find(algo_name);
        if (it == planner_entries_.end()) return nullptr;
        loader = it->second.second;
    }
    return loader(node, instance);
}

std::shared_ptr<IParamPollable> AlgoRegistry::loadMapParams(
    const std::string& algo_name, rclcpp::Node::SharedPtr node, void* instance)
{
    ConfigLoader loader;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = map_entries_.find(algo_name);
        if (it == map_entries_.end()) return nullptr;
        loader = it->second.second;
    }
    return loader(node, instance);
}

// ══════════════════════════════════════════════════════════════════════════════
//  Introspection
// ══════════════════════════════════════════════════════════════════════════════

std::vector<std::string> AlgoRegistry::plannerNames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return plannerNamesLocked();
}

std::vector<std::string> AlgoRegistry::mapNames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mapNamesLocked();
}

// ══════════════════════════════════════════════════════════════════════════════
//  Private helpers
// ══════════════════════════════════════════════════════════════════════════════

std::vector<std::string> AlgoRegistry::plannerNamesLocked() const {
    std::vector<std::string> names;
    names.reserve(planner_entries_.size());
    for (const auto& [name, _] : planner_entries_) names.push_back(name);
    std::sort(names.begin(), names.end());
    return names;
}

std::vector<std::string> AlgoRegistry::mapNamesLocked() const {
    std::vector<std::string> names;
    names.reserve(map_entries_.size());
    for (const auto& [name, _] : map_entries_) names.push_back(name);
    std::sort(names.begin(), names.end());
    return names;
}

std::string AlgoRegistry::joinNames(const std::vector<std::string>& names) {
    std::string joined;
    for (size_t i = 0; i < names.size(); ++i) {
        if (i) joined += ", ";
        joined += names[i];
    }
    return joined;
}

}  // namespace finenav::core
