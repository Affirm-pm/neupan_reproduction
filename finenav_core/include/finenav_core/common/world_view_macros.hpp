// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/common/world_view_registry.hpp"
#include "finenav_core/map/world_view_concept.hpp"

/**
 * @brief Registers a WorldView (a planner↔map bridge) into the framework.
 *
 * @param VIEW_TYPE     Concrete class deriving PLANNER_TYPE::IWorldView.
 *                      Must satisfy finenav::core::WorldView (enforced by a
 *                      static_assert below) and must be constructible as
 *                      ViewType(rclcpp::Node::SharedPtr, const Map1&, const Map2&, ...)
 *                      — the layer's private sub-node comes FIRST, then the maps.
 * @param PARAM_NS      Human-readable identifier for this WorldView.  Must match
 *                      the parameter namespace for it (the top-level key of its
 *                      .yaml parameter file).  Exactly as in
 *                      FINENAV_REGISTER_PLANNER / FINENAV_REGISTER_MAP, this one
 *                      token is both the registration name and the C++ namespace
 *                      the WorldView's Params / ParamListener come from.
 * @param PLANNER_TYPE  The planner whose IWorldView this view implements.
 * @param ...           One or more map types the view's constructor reads.
 *
 * The macro records only TYPES (not names) and a generated build function.  At
 * assembly time the framework resolves each type → algorithm name through
 * PlanRegistry / MapRegistry, so the WorldView author never writes a
 * planner/map name by hand.
 *
 * Like a Planner and a Map, every WorldView carries parameters: the macro wires
 * configure() to PARAM_NS, and additionally mints the type-erased prepare()
 * invoker the framework calls once per plan() inside the map read lock.
 */
#define FINENAV_REGISTER_WORLDVIEW(VIEW_TYPE, PARAM_NS, PLANNER_TYPE, ...) \
    namespace finenav::core { \
    template <> \
    struct detail::WorldViewRegistrationSlot<VIEW_TYPE> { \
        static_assert(WorldView<VIEW_TYPE, PLANNER_TYPE>, \
            "FINENAV_REGISTER_WORLDVIEW: VIEW_TYPE must satisfy finenav::core::WorldView " \
            "— it must declare ConfigType, configure(const ConfigType&) and " \
            "prepare(const PlanningContext<PlannerT>&)."); \
        inline static detail::WorldViewSelfRegistrar _world_view_registrar{ \
            #PARAM_NS, \
            WorldViewRegistry::Entry{ \
                detail::typeName<PLANNER_TYPE>(), \
                detail::typeNames<__VA_ARGS__>(), \
                [](MapAccess& maps, rclcpp::Node::SharedPtr node) -> BuiltWorldView { \
                    auto [view, prepare] = \
                        detail::makeWorldView<VIEW_TYPE, PLANNER_TYPE, __VA_ARGS__>(maps, node); \
                    auto params = detail::makeWorldViewParams< \
                        VIEW_TYPE, PARAM_NS::Params, PARAM_NS::ParamListener, PLANNER_TYPE>( \
                            node, view, #PARAM_NS); \
                    return BuiltWorldView{std::move(view), std::move(params), std::move(prepare)}; \
                }}}; \
    }; \
    }
