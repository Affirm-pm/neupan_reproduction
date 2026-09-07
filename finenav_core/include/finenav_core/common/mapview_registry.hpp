// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <typeinfo>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/common/named_registry.hpp"
#include "finenav_core/map/map_access.hpp"

namespace finenav::core {

/// One MapView's registry row: name → { planner type, map types, build }.
/// (No config loader — a MapView is not an algorithm, so it carries no params.)
struct MapViewEntry {
    using BuildFn = std::function<std::shared_ptr<void>(MapAccess&, rclcpp::Node::SharedPtr)>;

    std::string              planner_type;  ///< typeid(PlannerT).name()
    std::vector<std::string> map_types;     ///< typeid(MapT).name() per depended map
    BuildFn                  build;         ///< (MapAccess&, node) → erased IMapView
};

/**
 * @brief MapViewRegistry — runtime discovery table for planner↔map bridges.
 *
 * A MapView is a framework-independent bridge between a concrete map and a
 * planner's IMapView.  Its .so self-registers here via FINENAV_REGISTER_MAPVIEW,
 * declaring only the planner type + map types it bridges plus a build function.
 *
 * The framework later assembles a navigation system by name: it looks up the
 * entry, resolves the planner/map names by type through PlanRegistry /
 * MapRegistry, creates (and dedups) the MapServer(s) + ControlLayer, then
 * invokes build() to wire the view into the layer.
 *
 * Shares the RegistryTable bottom with PlanRegistry / MapRegistry, so the same
 * cross-DSO-singleton discipline + "keep-first" name semantics apply.
 */
class MapViewRegistry : public RegistryTable<MapViewEntry> {
public:
    using BuildFn = MapViewEntry::BuildFn;
    using Entry   = MapViewEntry;

    /// Non-inline function-local singleton defined in finenav_core.so so every
    /// DSO shares ONE table (same mechanism as PlanRegistry / MapRegistry).
    static MapViewRegistry& instance();

    /// Duplicate names keep the FIRST registrant (with a WARN).
    void registerEntry(std::string name, Entry entry);
};

namespace detail {

/// Mangled type name.  Unlike type_info addresses (which may differ per DSO
/// under -fvisibility=hidden), the mangled NAME is a compile-time constant that
/// is identical across plugin boundaries — used as the cross-plugin type id.
template <typename T>
std::string typeName() {
    return typeid(T).name();
}

template <typename... MapTs>
std::vector<std::string> typeNames() {
    return {typeid(MapTs).name()...};
}

/**
 * @brief Auto-build a MapView from its typed constructor
 *        ViewT(rclcpp::Node::SharedPtr, const Map1&, const Map2&, ...).
 *
 * The layer's private sub-node is handed in first (node-first convention); the
 * resolved maps follow as the constructor's map arguments.
 *
 * Upcasts to PlannerT::IMapView BEFORE erasing to void*, so the erased pointer
 * already points at the base subobject — ControlLayer's static cast back to
 * PlannerT::IMapView is then an identity cast (no undefined pointer adjustment).
 */
template <typename ViewT, typename PlannerT, typename... MapTs>
std::shared_ptr<void> makeMapView(MapAccess& maps, rclcpp::Node::SharedPtr node) {
    std::shared_ptr<typename PlannerT::IMapView> view =
        std::make_shared<ViewT>(node, maps.get<MapTs>()...);
    return view;
}

template <typename ViewT>
struct MapViewRegistrationSlot {};

struct MapViewSelfRegistrar {
    MapViewSelfRegistrar(std::string name, MapViewEntry entry) {
        MapViewRegistry::instance().registerEntry(std::move(name), std::move(entry));
    }
};

}  // namespace detail
}  // namespace finenav::core
