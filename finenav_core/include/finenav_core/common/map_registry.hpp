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
#include "finenav_core/map/i_map_resource.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/robot_state/robot_state.hpp"

namespace finenav::core {

template <Map MapT> class MapServer;  // full definition in map_server.hpp

/// One map's registry row: name → { factory, config loader, type }.
struct MapEntry {
    using Factory = std::function<std::shared_ptr<IMapResource>(
        rclcpp::Node::SharedPtr, const std::string&, RobotStateProvider*)>;

    Factory      factory;  ///< (node, server_name, state) → IMapResource
    ConfigLoader loader;   ///< (node, void* map) → IParamPollable
    std::string  type;     ///< typeid(MapT).name()
};

/**
 * @brief MapRegistry — the runtime reflection table for MAPS.
 *
 * A map .so self-registers here (via FINENAV_REGISTER_MAP): name →
 * { factory, config loader, type }.  The engine looks the factory up by name to
 * build a MapServer, and reverse-resolves type→name when assembling a WorldView.
 */
class MapRegistry : public RegistryTable<MapEntry> {
public:
    using Factory = MapEntry::Factory;
    using Entry   = MapEntry;

    static MapRegistry& instance();

    /// Self-registration: record the map factory + config loader under @p name.
    template <Map MapT>
    void registerMap(std::string name, ConfigLoader loader);

    /// Build a MapServer for the map registered under @p algo_name.
    /// Returns nullptr + ERROR log on unknown name.
    std::shared_ptr<IMapResource> createMap(
        rclcpp::Node::SharedPtr node,
        const std::string& algo_name, const std::string& server_name,
        RobotStateProvider* state);

    /// Look up the config loader registered under @p algo_name (nullptr if unknown).
    std::shared_ptr<IParamPollable> loadMapParams(
        const std::string& algo_name, rclcpp::Node::SharedPtr node, void* instance);

    std::vector<std::string> mapNames() const;

    /// Resolve the registered NAME for a concrete map type (typeid(MapT).name()).
    std::optional<std::string> findMapNameByType(const std::string& type_name) const;

private:
    void registerMapImpl(std::string name, Factory factory,
                         ConfigLoader loader, std::string type_name);
};

template <Map MapT>
void MapRegistry::registerMap(std::string name, ConfigLoader loader) {
    registerMapImpl(
        name,
        [name](rclcpp::Node::SharedPtr node, const std::string& server_name,
               RobotStateProvider* state) -> std::shared_ptr<IMapResource> {
            return std::make_shared<MapServer<MapT>>(node, name, server_name, state);
        },
        std::move(loader),
        typeid(MapT).name());
}

namespace detail {

/// Self-registrar for a map; instantiated as `inline static` in the macro.
template <typename T>
struct MapSelfRegistrar {
    explicit MapSelfRegistrar(std::string name, ConfigLoader loader) {
        MapRegistry::instance().registerMap<T>(std::move(name), std::move(loader));
    }
};

}  // namespace detail
}  // namespace finenav::core
