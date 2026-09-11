// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "dummy_map.hpp"
#include "dummy_planner.hpp"

namespace finenav::example {

/**
 * @brief SimpleWorldView — the bridge between DummyMap and DummyPlanner::IWorldView.
 *
 * This is the ONE artifact that knows both concrete types.  It is written once
 * by the navigation-logic author and registered via FINENAV_REGISTER_WORLDVIEW;
 * the end user then assembles the ControlLayer by NAME without ever seeing
 * either DummyMap or DummyPlanner.
 *
 * The constructor convention
 * (ViewT(rclcpp::Node::SharedPtr, const Map1&, const Map2&, ...)) is what the
 * framework uses to auto-build the view from the maps it resolved at assembly
 * time — the author only declares the map types in the registration macro.
 */
class SimpleWorldView : public DummyPlanner::IWorldView {
public:
    explicit SimpleWorldView(rclcpp::Node::SharedPtr node, const DummyMap& map) : map_(map) {
        (void)node;  // node 保留备用；纯只读桥当前不需要订阅
    }

    bool isWalkable(const finenav::core::Position3D& p) const override {
        return !map_.isOccupied(p);
    }

private:
    const DummyMap& map_;
};

}  // namespace finenav::example
