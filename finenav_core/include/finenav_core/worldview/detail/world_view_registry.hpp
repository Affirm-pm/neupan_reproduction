// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// Framework-internal WorldView discovery table.  Not part of the public API —
// plugin authors register a WorldView with FINENAV_REGISTER_WORLDVIEW; the
// engine assembles a layer by name through
// FineNavEngine::createLayerByWorldView().

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_common/detail/named_registry.hpp"
#include "finenav_core/detail/algo_register_details.hpp"
#include "finenav_core/map/map_access.hpp"
#include "finenav_core/plan/i_pipeline_node.hpp"
#include "finenav_core/plan/planner_concept.hpp"  // PlanningContext<PlannerT>

namespace finenav::core::detail {

/// One WorldView's registry row: name → { planner type, map types, build }.
struct WorldViewEntry {
    using BuildFn = std::function<BuiltWorldView(MapAccess&, rclcpp::Node::SharedPtr)>;

    std::string              planner_type;  ///< typeid(PlannerT).name()
    std::vector<std::string> map_types;     ///< typeid(MapT).name() per depended map
    BuildFn                  build;         ///< (MapAccess&, node) → BuiltWorldView
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
 * Shares the common::detail::RegistryTable bottom with PlanRegistry /
 * MapRegistry, so the same cross-DSO-singleton discipline + "keep-first" name
 * semantics apply.
 */
class WorldViewRegistry : public common::detail::RegistryTable<WorldViewEntry> {
public:
    using BuildFn = WorldViewEntry::BuildFn;
    using Entry   = WorldViewEntry;

    /// Non-inline function-local singleton defined in finenav_core.so so every
    /// DSO shares ONE table (same mechanism as PlanRegistry / MapRegistry).
    static WorldViewRegistry& instance();

    /// Duplicate names keep the FIRST registrant (with a WARN).
    void registerEntry(std::string name, Entry entry);
};

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
 * already points at the base subobject — PipelineNode's static cast back to
 * PlannerT::IWorldView is then an identity cast (no undefined pointer adjustment).
 *
 * Also mints the type-erased prepare() invoker.  It has to be created HERE,
 * where ViewT is concrete: downstream the view's static type is only
 * PlannerT::IWorldView, so prepare() would be undetectable there.
 *
 * @return {erased view, erased prepare invoker}
 */
template <typename ViewT, typename PlannerT, typename... MapTs>
std::pair<std::shared_ptr<void>, BuiltWorldView::PrepareFn>
makeWorldView(MapAccess& maps, rclcpp::Node::SharedPtr node) {
    auto concrete = std::make_shared<ViewT>(node, maps.get<MapTs>()...);
    // The view outlives this invoker (both are owned by the same PipelineNode,
    // and prepare_ is only ever called while view_ is set), so a raw borrow is
    // enough — no need to keep a second shared_ptr alive.
    ViewT* borrowed = concrete.get();
    BuiltWorldView::PrepareFn prepare = [borrowed](const void* ctx) {
        borrowed->prepare(*static_cast<const PlanningContext<PlannerT>*>(ctx));
    };
    std::shared_ptr<typename PlannerT::IWorldView> view = std::move(concrete);
    return {std::move(view), std::move(prepare)};
}

/**
 * @brief Wire a WorldView's parameter channel from its generate_parameter_library
 *        YAML: runs configure(params) once, re-runs it on every `ros2 param set`,
 *        and duck-type injects setLogger() / setVisualizer() when declared.
 *
 * The erased pointer targets the PlannerT::IWorldView base subobject, so it must
 * be cast back down to ViewT before the concrete configure() can be called.
 */
template <typename ViewT, typename ParamsT, typename ListenerT, typename PlannerT>
std::shared_ptr<common::IParamPollable> makeWorldViewParams(
    rclcpp::Node::SharedPtr node, const std::shared_ptr<void>& view, std::string_view name) {
    auto* base = static_cast<typename PlannerT::IWorldView*>(view.get());
    auto* concrete = static_cast<ViewT*>(base);
    return makeParamPollable<ViewT, ParamsT, ListenerT>(node, *concrete, name);
}

struct WorldViewSelfRegistrar {
    WorldViewSelfRegistrar(std::string name, WorldViewEntry entry) {
        WorldViewRegistry::instance().registerEntry(std::move(name), std::move(entry));
    }
};

}  // namespace finenav::core::detail
