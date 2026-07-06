// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "finenav_engine/finenav_engine.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "dummy_map_params.hpp"

// ── DummyMap — a minimal ShiftableMap ───────────────────────────────────────

struct DummyCell {};

struct DummyMap {
    using ConfigType = dummy_map::Params;
    using DataType   = DummyCell;

    DummyMap() = default;

    void configure(const ConfigType&) {}

    bool isInside(const finenav::core::Position3D& p) const {
        return bounds_.contains(p);
    }
    finenav::core::Region3D getWindowBounds() const { return bounds_; }
    finenav::core::Position3D getWindowCenter() const { return bounds_.center(); }
    void shiftWindowTo(const finenav::core::Position3D& c) {
        auto h = bounds_.sizes() * 0.5;
        bounds_ = finenav::core::Region3D(c - h, c + h);
    }

private:
    finenav::core::Region3D bounds_{
        finenav::core::Position3D(-50, -50, -5),
        finenav::core::Position3D( 50,  50,  5)};
};

static_assert(finenav::core::SpatialContainer<DummyMap>);
static_assert(finenav::core::ShiftableMap<DummyMap>);

FINENAV_REGISTER_ALGO_CONFIG(DummyMap, dummy_map)

// ── Mock state provider ─────────────────────────────────────────────────────

struct MockProvider : finenav::core::IRobotStateProvider {
    finenav::core::RobotState state{};
    std::optional<finenav::core::RobotState> getRobotState() const override {
        return state;
    }
    std::optional<finenav::core::RobotState> getRobotState(
        const rclcpp::Time&) const override { return state; }
};

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("create_custom_map");
    MockProvider mock;

    // Instantiate MapServer with the custom map
    auto server = std::make_shared<finenav::core::MapServer<DummyMap>>(
        node, "dummy_map_server", &mock);

    RCLCPP_INFO(node->get_logger(),
        "DummyMap registered successfully. Window centre: (%.1f, %.1f, %.1f)",
        server->getLockedReadView()->getWindowCenter().x(),
        server->getLockedReadView()->getWindowCenter().y(),
        server->getLockedReadView()->getWindowCenter().z());

    rclcpp::shutdown();
    return 0;
}
