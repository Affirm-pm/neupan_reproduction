// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <concepts>
#include <utility>

#include "finenav_core/math_def.hpp"

namespace finenav::core {

// ── Forward declaration ──────────────────────────────────────────────────────
// MapRuntime is non-template — no constraint needed.
class MapRuntime;

/**
 * @brief Map — the universal contract for all maps.
 *
 * Every Map MUST:
 *   - Be a bounded spatial container (isInside, getWindowBounds, getWindowCenter)
 *   - Accept YAML configuration via configure(ConfigType)
 *   - Wire its own data sources and lifecycle hooks via selfManage(MapRuntime&)
 *
 * There is no "dumb container" option.  Every Map author declares what sensor
 * data the map needs, what pre/post-update processing to run, and how to
 * configure itself — all inside selfManage().  Users just select a Map type
 * and the framework handles the rest.
 *
 * Map developers interact ONLY with MapRuntime — they never see MapServer.
 */
template <typename T>
concept Map =
    std::default_initializable<T> &&
    requires(T map, const Position3D& pos,
             const typename T::ConfigType& config,
             MapRuntime& runtime) {
        typename T::ConfigType;
        typename T::DataType;

        // --- Configuration (from YAML params) ---
        { map.configure(config) } -> std::same_as<void>;

        // --- Window queries ---
        { std::as_const(map).isInside(pos) }      -> std::same_as<bool>;
        { std::as_const(map).getWindowBounds() }  -> std::convertible_to<Region3D>;
        { std::as_const(map).getWindowCenter() }  -> std::convertible_to<Position3D>;

        // --- Self-wiring (declares data sources, hooks, update logic) ---
        { map.selfManage(runtime) } -> std::same_as<void>;
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
