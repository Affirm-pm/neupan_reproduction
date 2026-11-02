// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/plugin_world_view.hpp"

#include "dummy_follow_planner.hpp"
#include "dummy_line_planner.hpp"
#include "dummy_map.hpp"
#include "finenav_example/dummy_follow_world_view_params.hpp"
#include "finenav_example/dummy_line_world_view_params.hpp"

namespace finenav::example {

class DummyLineWorldView : public DummyLinePlanner::IWorldView {
  public:
    using ConfigType = dummy_line_world_view::Params;

    DummyLineWorldView(rclcpp::Node::SharedPtr, const DummyMap& map) : map_(map) {}

    void configure(const ConfigType&) {}

    void prepare(const finenav::core::PlanningContext<DummyLinePlanner>&) {}

    bool isWalkable(const finenav::common::Position3D& position) const override { return !map_.isOccupied(position); }

  private:
    const DummyMap& map_;
};

class DummyFollowWorldView : public DummyFollowPlanner::IWorldView {
  public:
    using ConfigType = dummy_follow_world_view::Params;

    DummyFollowWorldView(rclcpp::Node::SharedPtr, const DummyMap& map) : map_(map) {}

    void configure(const ConfigType&) {}

    void prepare(const finenav::core::PlanningContext<DummyFollowPlanner>&) {}

    bool isWalkable(const finenav::common::Position3D& position) const override { return !map_.isOccupied(position); }

  private:
    const DummyMap& map_;
};

}  // namespace finenav::example
