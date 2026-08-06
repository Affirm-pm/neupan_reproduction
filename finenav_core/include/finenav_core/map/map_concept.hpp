// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <concepts>
#include <utility>

#include "finenav_core/math_def.hpp"

namespace finenav::core {

/**
 * @brief Map — the spatial-container contract for all maps.
 *
 * A Map is a bounded spatial data container.  It is deliberately a "dumb"
 * contract — it only describes the data structure and queries.  It does NOT
 * know about ROS, data sources, or update cycles.
 *
 * Map authors who want the map to wire itself can additionally satisfy
 * SelfManagingMap.
 */
template <typename T>
concept Map =
    std::default_initializable<T> &&
    requires(T map, const Position3D& pos,
             const typename T::ConfigType& config) {
        typename T::ConfigType;
        typename T::DataType;

        // --- Basic configuration (from YAML params) ---
        { map.configure(config) } -> std::same_as<void>;

        // --- Window queries ---
        { std::as_const(map).isInside(pos) }      -> std::same_as<bool>;
        { std::as_const(map).getWindowBounds() }  -> std::convertible_to<Region3D>;
        { std::as_const(map).getWindowCenter() }  -> std::convertible_to<Position3D>;
    };

// ── Map is now defined; forward-declare MapServer with the constraint ────────
template <Map MapT> class MapServer;

/**
 * @brief SelfManagingMap — a Map that wires its own data sources and lifecycle.
 *
 * MapServer detects this at compile time (if constexpr).  If satisfied, the
 * map's selfManage() is called during construction so the map can declare
 * its own observation sources, hooks, and update logic via the server's API.
 * Otherwise the user wires everything manually.
 */
template <typename T>
concept SelfManagingMap = Map<T> &&
    requires(T& map, MapServer<T>& server) {
        { map.selfManage(server) } -> std::same_as<void>;
    };

/**
 * @brief ShiftableMap — a Map whose window can follow the robot.
 */
template <typename T>
concept ShiftableMap = Map<T> &&
    requires(T map, const Position3D& pos) {
        { map.shiftWindowTo(pos) } -> std::same_as<void>;
    };

}  // namespace finenav::core
