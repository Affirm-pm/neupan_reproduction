// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/common/mapview_registry.hpp"

/**
 * @brief Registers a MapView (a planner↔map bridge) into the framework.
 *
 * @param MAPVIEW_NAME  Name the downstream user selects in config/YAML.
 * @param VIEW_TYPE     Concrete class deriving PlannerT::IMapView.
 * @param PLANNER_TYPE  The planner whose IMapView this view implements.
 * @param ...           One or more map types the view's constructor reads.
 *                      The constructor must be
 *                      ViewType(rclcpp::Node::SharedPtr, const Map1&, const Map2&, ...)
 *                      — the layer's private sub-node comes FIRST, then the maps.
 *
 * The macro records only TYPES (not names) and a generated build function.  At
 * assembly time the framework resolves each type → algorithm name through
 * PlanRegistry / MapRegistry, so the MapView author never writes a
 * planner/map name by hand.
 */
#define FINENAV_REGISTER_MAPVIEW(MAPVIEW_NAME, VIEW_TYPE, PLANNER_TYPE, ...) \
    namespace finenav::core { \
    template <> \
    struct detail::MapViewRegistrationSlot<VIEW_TYPE> { \
        inline static detail::MapViewSelfRegistrar _mapview_registrar{ \
            #MAPVIEW_NAME, \
            MapViewRegistry::Entry{ \
                detail::typeName<PLANNER_TYPE>(), \
                detail::typeNames<__VA_ARGS__>(), \
                [](MapAccess& maps, rclcpp::Node::SharedPtr node) -> std::shared_ptr<void> { \
                    return detail::makeMapView<VIEW_TYPE, PLANNER_TYPE, __VA_ARGS__>(maps, node); \
                }}}; \
    }; \
    }
