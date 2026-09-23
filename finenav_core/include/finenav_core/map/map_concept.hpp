// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <concepts>
#include <utility>

#include "finenav_common/geometry_types.hpp"

namespace finenav::core {

// ── Forward declaration ──────────────────────────────────────────────────────
// MapRuntime is non-template — no constraint needed.
class MapRuntime;

/**
 * @brief Map — the universal contract for all maps.
 *
 * Every Map MUST:
 *   - Accept YAML configuration via configure(ConfigType)
 *   - Wire its own data sources and lifecycle hooks via selfManage(MapRuntime&)
 *
 * There is no "dumb container" option.  Every Map author declares what sensor
 * data the map needs, what pre/post-update processing to run, and how to
 * configure itself — all inside selfManage().  Users just select a Map type
 * and the framework handles the rest.
 *
 * A Map is the WORLD: objective data, independent of who is looking at it.
 * Being bounded/windowed is NOT part of this contract — that capability lives
 * in ShiftableMap, and any spatial structure can be asked for its extent.
 *
 * Map developers interact ONLY with MapRuntime — they never see MapServer.
 */
template <typename T>
concept Map =
    std::default_initializable<T> &&
    requires(T map,
             const typename T::ConfigType& config,
             MapRuntime& runtime) {
        typename T::ConfigType;
        typename T::DataType;

        // --- Configuration (from YAML params) ---
        { map.configure(config) } -> std::same_as<void>;

        // --- Self-wiring (declares data sources, hooks, update logic) ---
        { map.selfManage(runtime) } -> std::same_as<void>;
    };

/**
 * @brief ShiftableMap — a Map that carries a bounded window which can follow
 *        the robot.
 *
 * Adds the window queries on top of Map.  Only maps whose content is kept in a
 * rolling window around the robot need this: MapServer uses these queries to
 * detect the robot escaping the window and to re-centre it.
 */
template <typename T>
concept ShiftableMap = Map<T> &&
    requires(T map, const common::Position3D& pos) {
        { std::as_const(map).isInside(pos) }     -> std::same_as<bool>;
        { std::as_const(map).getWindowBounds() } -> std::convertible_to<common::Region3D>;
        { std::as_const(map).getWindowCenter() } -> std::convertible_to<common::Position3D>;
        { map.shiftWindowTo(pos) }               -> std::same_as<void>;
    };

}  // namespace finenav::core
