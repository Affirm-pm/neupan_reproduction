// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "finenav_navigator/chassis_command/i_chassis_command.hpp"

namespace finenav::navigator {

class ChassisCmdVelStamped : public IChassisCommand {
public:
    ChassisCmdVelStamped(rclcpp::Node::SharedPtr node,
                         const std::string& topic = "/cmd_vel");
    bool validate(const finenav_interfaces::msg::Trajectory& traj) const override;
    void publish(const finenav_interfaces::msg::Trajectory& traj) override;
    void stop() override;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> pub_;
};

}  // namespace finenav::navigator
