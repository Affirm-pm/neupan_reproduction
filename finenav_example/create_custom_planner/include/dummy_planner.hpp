// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include <geometry_msgs/msg/twist.hpp>

#include "finenav_core/algo_configurator.hpp"
#include "finenav_core/math_def.hpp" // TODO： 应该丢掉
#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/plan/trajectory.hpp" // TODO： 应该合并

#include "finenav_example/dummy_planner_params.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

namespace finenav::example {

class DummyMapView {
public:
    virtual bool isWalkable(const finenav::core::Position3D&) const = 0;
    virtual ~DummyMapView() = default;
};

/**
 * @brief DummyPlanner — a minimal example planner demonstrating the parameter pattern.
 *
 * === Parameter hot-reload ===
 *
 * DummyPlanner's parameters (linear_speed, angular_speed) are hot-reloadable
 * because their YAML does NOT have `read_only: true`.  The framework calls
 * configure() in two scenarios:
 *   1. Startup:  AlgoConfigurator loads initial values (YAML defaults or
 *               --params-file overrides).
 *   2. Runtime:  `ros2 param set` triggers the ParamListener callback, which
 *               calls configure() with the full new parameter set.
 *
 * The same configure() handles both — check the .cpp for details.
 *
 * If any parameter marked read_only: true, configure() will only be called at
 * startup and ros2 param set will be rejected by generate_parameter_library.
 */
class DummyPlanner {
public:
    using ConfigType    = dummy_planner::Params;
    using IMapView      = DummyMapView;

    static constexpr finenav::core::IOProfile InputProfile{false, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{false, true, false, false};

    DummyPlanner();

    /**
     * @brief Apply parameter configuration.
     * @details
     * Called by the framework at startup AND on every ros2 param update.
     * If you do not want a parameter to support hot update, simply add `read_only: true` to its YAML entry.
     * @param cfg  Full parameter struct (generated from YAML).
     */
    void configure(const ConfigType& cfg);

    // ── Framework injectors ─────────────────────────────────────────
    // Declare these to receive the logger and visualizer automatically.
    // The framework detects them via if constexpr — no extra registration needed.
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }
    void setVisualizer(finenav::viz::FineNavVisualizer& viz) { viz_ = &viz; } // TODO：这里的签名怎么不是const

    void reset();

    using Context = finenav::core::PlanningContext<DummyPlanner>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<DummyPlanner>>;

    Result plan(const Context& ctx,const IMapView& map_view);

private:
    double linear_speed_  = 0.5;
    double angular_speed_ = 1.0;
    rclcpp::Logger                logger_{rclcpp::get_logger("DummyPlanner")};
    finenav::viz::FineNavVisualizer* viz_ = nullptr;
};

}  // namespace finenav::example

// Register this planner into the FineNav-Engine parameter system.
// "dummy_planner" must match the top-level key in dummy_planner.yaml.
FINENAV_REGISTER_ALGO_CONFIG(finenav::example::DummyPlanner, dummy_planner)
