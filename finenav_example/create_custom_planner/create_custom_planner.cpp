// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/plan/trajectory.hpp"
#include "finenav_core/plan/planner_set.hpp"
#include "finenav_core/plan/control_layer.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "dummy_planner_params.hpp"

// ── DummyPlanner — minimal Planner implementation ───────────────────────────

struct DummyIMapView {};

struct DummyPlanner {
    using ConfigType    = dummy_planner::Params;
    using IMapView      = DummyIMapView;

    static constexpr finenav::core::IOProfile InputProfile{false, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    void configure(const ConfigType&) {}
    void reset() {}

    std::optional<finenav::core::Trajectory<OutputProfile>> plan(
        const finenav::core::PlanningContext<DummyPlanner>&,
        const DummyIMapView&) {
        finenav::core::Trajectory<OutputProfile> result;
        result.poses.push_back(geometry_msgs::msg::Pose{});
        return result;
    }
};

static_assert(finenav::core::Planner<DummyPlanner>);

FINENAV_REGISTER_ALGO_CONFIG(DummyPlanner, dummy_planner)

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("create_custom_planner");

    using Planners = finenav::core::PlannerSet<DummyPlanner>;
    auto layer = std::make_shared<finenav::core::ControlLayer<Planners>>(
        node, "dummy_layer");

    RCLCPP_INFO(node->get_logger(), "DummyPlanner registered successfully.");
    rclcpp::shutdown();
    return 0;
}
