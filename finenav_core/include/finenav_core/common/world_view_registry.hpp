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

/// One WorldView's registry row: name → { planner type, map types, build }.
/// (No config loader — a WorldView is not an algorithm, so it carries no params.)
struct WorldViewEntry {
    using BuildFn = std::function<std::shared_ptr<void>(MapAccess&, rclcpp::Node::SharedPtr)>;

    std::string              planner_type;  ///< typeid(PlannerT).name()
    std::vector<std::string> map_types;     ///< typeid(MapT).name() per depended map
    BuildFn                  build;         ///< (MapAccess&, node) → erased IWorldView
};

/**
 * @brief WorldViewRegistry — runtime discovery table for planner↔map bridges.
 *
 * A WorldView is a framework-independent bridge between a concrete map and a
 * planner's IWorldView.  Its .so self-registers here via FINENAV_REGISTER_WORLDVIEW,
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
class WorldViewRegistry : public RegistryTable<WorldViewEntry> {
public:
    using BuildFn = WorldViewEntry::BuildFn;
    using Entry   = WorldViewEntry;

    /// Non-inline function-local singleton defined in finenav_core.so so every
    /// DSO shares ONE table (same mechanism as PlanRegistry / MapRegistry).
    static WorldViewRegistry& instance();

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
 * @brief Auto-build a WorldView from its typed constructor
 *        ViewT(rclcpp::Node::SharedPtr, const Map1&, const Map2&, ...).
 *
 * The layer's private sub-node is handed in first (node-first convention); the
 * resolved maps follow as the constructor's map arguments.
 *
 * Upcasts to PlannerT::IWorldView BEFORE erasing to void*, so the erased pointer
 * already points at the base subobject — ControlLayer's static cast back to
 * PlannerT::IWorldView is then an identity cast (no undefined pointer adjustment).
 */
template <typename ViewT, typename PlannerT, typename... MapTs>
std::shared_ptr<void> makeWorldView(MapAccess& maps, rclcpp::Node::SharedPtr node) {
    std::shared_ptr<typename PlannerT::IWorldView> view =
        std::make_shared<ViewT>(node, maps.get<MapTs>()...);
    return view;
}

template <typename ViewT>
struct WorldViewRegistrationSlot {};

struct WorldViewSelfRegistrar {
    WorldViewSelfRegistrar(std::string name, WorldViewEntry entry) {
        WorldViewRegistry::instance().registerEntry(std::move(name), std::move(entry));
    }
};

}  // namespace detail
}  // namespace finenav::core
