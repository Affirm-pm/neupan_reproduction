// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <concepts>
#include <utility>

#include "finenav_core/math_def.hpp"

namespace finenav::core {

/**
 * @brief SpatialContainer — the minimal contract for any bounded spatial data container.
 *
 */
template <typename T>
concept SpatialContainer =
    std::default_initializable<T> &&
    requires(T map, const Position3D& pos,
             const typename T::ConfigType& config,
             const typename T::DataType& data) {
        typename T::ConfigType;
        typename T::DataType;

        // --- Basic configuration ---
        { map.configure(config) } -> std::same_as<void>;

        // --- Window queries ---
        { std::as_const(map).isInside(pos) }      -> std::same_as<bool>;
        { std::as_const(map).getWindowBounds() }  -> std::convertible_to<Region3D>;
        { std::as_const(map).getWindowCenter() }  -> std::convertible_to<Position3D>;
    };


/**
 * @brief Shiftable Map
 */
template <typename T>
concept ShiftableMap = SpatialContainer<T> &&
    requires(T map, const Position3D& pos) {
        { map.shiftWindowTo(pos) } -> std::same_as<void>;
    };

}  // namespace finenav::core
