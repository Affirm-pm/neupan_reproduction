// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/common/world_view_registry.hpp"

/**
 * @brief Registers a WorldView (a planner↔map bridge) into the framework.
 *
 * @param WORLDVIEW_NAME  Name the downstream user selects in config/YAML.
 * @param VIEW_TYPE     Concrete class deriving PlannerT::IWorldView.
 * @param PLANNER_TYPE  The planner whose IWorldView this view implements.
 * @param ...           One or more map types the view's constructor reads.
 *                      The constructor must be
 *                      ViewType(rclcpp::Node::SharedPtr, const Map1&, const Map2&, ...)
 *                      — the layer's private sub-node comes FIRST, then the maps.
 *
 * The macro records only TYPES (not names) and a generated build function.  At
 * assembly time the framework resolves each type → algorithm name through
 * PlanRegistry / MapRegistry, so the WorldView author never writes a
 * planner/map name by hand.
 */
#define FINENAV_REGISTER_WORLDVIEW(WORLDVIEW_NAME, VIEW_TYPE, PLANNER_TYPE, ...) \
    namespace finenav::core { \
    template <> \
    struct detail::WorldViewRegistrationSlot<VIEW_TYPE> { \
        inline static detail::WorldViewSelfRegistrar _world_view_registrar{ \
            #WORLDVIEW_NAME, \
            WorldViewRegistry::Entry{ \
                detail::typeName<PLANNER_TYPE>(), \
                detail::typeNames<__VA_ARGS__>(), \
                [](MapAccess& maps, rclcpp::Node::SharedPtr node) -> std::shared_ptr<void> { \
                    return detail::makeWorldView<VIEW_TYPE, PLANNER_TYPE, __VA_ARGS__>(maps, node); \
                }}}; \
    }; \
    }
