// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "finenav_navigator/chassis_command/i_chassis_command.hpp"

namespace finenav::navigator {

class ChassisCmdVel : public IChassisCommand {
public:
    ChassisCmdVel(rclcpp::Node::SharedPtr node,
                  const std::string& topic = "/cmd_vel");
    bool validate(const finenav_interfaces::msg::Trajectory& traj) const override;
    void publish(const finenav_interfaces::msg::Trajectory& traj) override;

private:
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> pub_;
};

}  // namespace finenav::navigator
