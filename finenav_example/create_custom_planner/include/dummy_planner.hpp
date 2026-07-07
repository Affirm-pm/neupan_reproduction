// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include <geometry_msgs/msg/twist.hpp>

#include "finenav_core/algo_configurator.hpp"
#include "finenav_core/math_def.hpp"
#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/plan/trajectory.hpp"

#include "finenav_example/dummy_planner_params.hpp"

namespace finenav::example {

class DummyMapView {
public:
    virtual bool isWalkable(const finenav::core::Position3D&) const = 0;
    virtual ~DummyMapView() = default;
};

class DummyPlanner {
public:
    using ConfigType    = dummy_planner::Params;
    using IMapView      = DummyMapView;

    static constexpr finenav::core::IOProfile InputProfile{false, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{false, true, false, false};

    DummyPlanner();

    void configure(const ConfigType& cfg);
    void reset();


    using Context = finenav::core::PlanningContext<DummyPlanner>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<DummyPlanner>>;

    Result plan(const Context& ctx,const IMapView& map_view);

private:
    double linear_speed_  = 0.5;
    double angular_speed_ = 1.0;
};

}  // namespace finenav::example

FINENAV_REGISTER_ALGO_CONFIG(finenav::example::DummyPlanner, dummy_planner)
