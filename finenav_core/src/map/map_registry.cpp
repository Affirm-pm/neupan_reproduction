// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_core/map/detail/map_registry.hpp"

#include "finenav_util/common/log_macros.hpp"

namespace finenav::core::detail {

// Lazy function-local static defined in finenav_core.so — the single shared
// map table across every plugin DSO (see FINENAV_DEFINE_REGISTRY_SINGLETON).
FINENAV_DEFINE_REGISTRY_SINGLETON(MapRegistry)

void MapRegistry::registerMapImpl(std::string name, Factory factory,
                                  ConfigLoader loader, std::string type_name) {
    bool inserted = insertOrKeep(name, MapEntry{
        std::move(factory), std::move(loader), std::move(type_name)});
    if (!inserted) {
        FINENAV_LOG_WARN(rclcpp::get_logger("MapRegistry"),
            "Map '{}' already registered — keeping the FIRST registrant "
            "(renaming one of the duplicate parameter namespaces resolves this).",
            name);
    } else {
        FINENAV_LOG_DEBUG(rclcpp::get_logger("MapRegistry"),
            "Map '{}' registered.", name);
    }
}

std::shared_ptr<IMapResource> MapRegistry::createMap(
    rclcpp::Node::SharedPtr node,
    const std::string& algo_name, const std::string& server_name,
    RobotStateProvider* state)
{
    auto entry = lookup(algo_name);
    if (!entry) {
        FINENAV_LOG_ERROR(rclcpp::get_logger("MapRegistry"),
            "createMap: map '{}' is not registered. "
            "Available maps: [{}]",
            algo_name, joinNames(names()));
        return nullptr;
    }
    return entry->factory(node, server_name, state);
}

std::shared_ptr<IParamPollable> MapRegistry::loadMapParams(
    const std::string& algo_name, rclcpp::Node::SharedPtr node, void* instance)
{
    auto entry = lookup(algo_name);
    if (!entry) return nullptr;
    return entry->loader(node, instance);
}

std::vector<std::string> MapRegistry::mapNames() const {
    return names();
}

std::optional<std::string> MapRegistry::findMapNameByType(
    const std::string& type_name) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& [name, entry] : entries_) {
        if (entry.type == type_name) return name;
    }
    return std::nullopt;
}

}  // namespace finenav::core::detail
