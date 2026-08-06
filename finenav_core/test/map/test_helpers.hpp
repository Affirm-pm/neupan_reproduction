// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Shared test infrastructure for finenav_core/map unit tests.

#pragma once

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_core/algo_configurator.hpp"

// ── Minimal self-managing Map for testing ───────────────────────────────────

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

    // selfManage — minimal no-op for tests that don't need data sources
    void selfManage(finenav::core::MapRuntime&) {}

private:
    finenav::core::Region3D bounds_{
        finenav::core::Position3D(-50, -50, -5),
        finenav::core::Position3D( 50,  50,  5)};
};

static_assert(finenav::core::Map<SmokeMap>);
static_assert(finenav::core::ShiftableMap<SmokeMap>);

// ── AlgoConfigurator specialisation ────────────────────────────────────────

namespace finenav::core {
template <>
struct AlgoConfigurator<SmokeMap> {
    static std::shared_ptr<IParamPollable> load(rclcpp::Node::SharedPtr, SmokeMap&) {
        return nullptr;
    }
};
} // namespace finenav::core

// ── Mock state provider ────────────────────────────────────────────────────

struct MockStateProvider : finenav::core::RobotStateProvider {
    finenav::core::RobotState state{};

    std::optional<finenav::core::RobotState> getRobotState() const override {
        return state;
    }
    std::optional<finenav::core::RobotState> getRobotState(
        const rclcpp::Time&) const override {
        return state;
    }
};
