// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/math_def.hpp"

#include "finenav_example/dummy_map_params.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

namespace finenav::example {

/**
 * @brief DummyMap — a minimal example map demonstrating read-only parameters.
 *
 * === Read-only parameters ===
 *
 * DummyMap's window_size is marked `read_only: true` in dummy_map.yaml.
 * This means:
 *   - configure() is called ONCE at startup (from YAML defaults or --params-file).
 *   - `ros2 param set` at runtime will be REJECTED by generate_parameter_library.
 *
 * Use read_only for parameters whose change would require rebuilding internal
 * data structures, reallocating memory, or otherwise cannot be safely applied
 * while the planner is running.
 *
 * For hot-reloadable parameters (like DummyPlanner's speeds), simply omit
 * read_only — the framework will call configure() on every ros2 param set.
 */
class DummyMap {
public:
    using ConfigType = dummy_map::Params;
    using DataType   = int;

    DummyMap();

    void configure(const ConfigType& cfg);

    // ── Framework injectors ─────────────────────────────────────────
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }
    void setVisualizer(const finenav::viz::FineNavVisualizer& viz) { viz_ = viz; }

    bool isInside(const finenav::core::Position3D& p) const;
    finenav::core::Region3D   getWindowBounds()  const;
    finenav::core::Position3D getWindowCenter()  const;

    // Inline so the getting_started executable needs no symbol from this .so —
    // it recovers the typed handle by header + RTTI only (the .so is dlopened).
    bool isOccupied(const finenav::core::Position3D&) const { return false; }

    void selfManage(finenav::core::MapRuntime&) {}

private:
    double window_size_ = 50.0;
    finenav::core::Region3D bounds_;
    rclcpp::Logger                logger_{rclcpp::get_logger("DummyMap")};
    finenav::viz::FineNavVisualizer viz_;
};

}  // namespace finenav::example
