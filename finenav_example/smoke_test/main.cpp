// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Minimal compile-check: instantiate MapServer with an inline SpatialContainer.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/math_def.hpp"
#include "finenav_core/algo_configurator.hpp"

// ── Minimal SpatialContainer for smoke testing ───────────────────────────────

struct SmokeCell { double value = 0.0; };

struct SmokeMap {
    using ConfigType = int;
    using DataType   = SmokeCell;

    void configure(const int&) {}

    bool isInside(const finenav::core::Position3D& p) const {
        return bounds_.contains(p);
    }
    finenav::core::Region3D getWindowBounds() const { return bounds_; }
    finenav::core::Position3D getWindowCenter() const { return bounds_.center(); }

    void shiftWindowTo(const finenav::core::Position3D& center) {
        auto half = bounds_.sizes() * 0.5;
        bounds_ = finenav::core::Region3D(center - half, center + half);
    }

private:
    finenav::core::Region3D bounds_{
        finenav::core::Position3D(-50, -50, -5),
        finenav::core::Position3D( 50,  50,  5)};
};

static_assert(finenav::core::SpatialContainer<SmokeMap>);
static_assert(finenav::core::ShiftableMap<SmokeMap>);

// ── AlgoConfigurator specialisation (required by MapServer constructor) ──────

namespace finenav::core {
template <>
struct AlgoConfigurator<SmokeMap> {
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr, SmokeMap&) {
        return nullptr;
    }
};
} // namespace finenav::core

// ── Mock state provider ─────────────────────────────────────────────────────

struct MockStateProvider : finenav::core::IRobotStateProvider {
    std::optional<finenav::core::RobotState> getRobotState() const override {
        return finenav::core::RobotState{};
    }
    std::optional<finenav::core::RobotState> getRobotState(
        const rclcpp::Time&) const override {
        return finenav::core::RobotState{};
    }
};

// ── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("smoke_test");
    MockStateProvider mock_provider;

    auto server = std::make_shared<finenav::core::MapServer<SmokeMap>>(
        node, "smoke_map_server", &mock_provider);

    auto ro_view = server->getLockedReadView();
    (void)ro_view->isInside(finenav::core::Position3D::Zero());

    auto rw_view = server->getLockedWriteView();
    (void)rw_view->getWindowCenter();

    auto injector = server->addObservationSource<SmokeCell>(
        "test_source",
        {.max_buffer_size = 32},
        [](const SmokeCell& cell, const rclcpp::Time& stamp,
           const finenav::core::RobotState& state, SmokeMap& map) {
            (void)cell;
            (void)stamp;
            (void)state;
            (void)map;
        });
    injector(SmokeCell{}, node->now());

    RCLCPP_INFO(node->get_logger(), "MapServer smoke test passed.");

    rclcpp::shutdown();
    return 0;
}
