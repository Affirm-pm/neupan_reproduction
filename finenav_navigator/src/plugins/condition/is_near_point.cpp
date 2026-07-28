// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/condition/is_near_point.hpp"

#include <cmath>
#include <behaviortree_cpp/bt_factory.h>

namespace finenav::navigator {

IsNearPoint::IsNearPoint(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList IsNearPoint::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Pose>("point_a",           "{robot_pose}", "First pose"),
        BT::InputPort<geometry_msgs::msg::Pose>("point_b",           "{goal}",       "Second pose"),
        BT::InputPort<double>("distance_threshold", 3.0, "Distance threshold [m]"),
    };
}

BT::NodeStatus IsNearPoint::tick() {
    geometry_msgs::msg::Pose a, b;
    double threshold = 3.0;

    if (!getInput("point_a", a) || !getInput("point_b", b)) {
        return BT::NodeStatus::FAILURE;
    }
    getInput("distance_threshold", threshold);

    const double dx   = a.position.x - b.position.x;
    const double dy   = a.position.y - b.position.y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    return (dist < threshold) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::IsNearPoint>("IsNearPoint");
}
