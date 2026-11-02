// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/stop_chassis_motion.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include "finenav_navigator/chassis_command/i_chassis_command.hpp"

namespace finenav::navigator {

BT::PortsList StopChassisMotion::providedPorts() {
    return {
        BT::InputPort<IChassisCommand*>("chassis_cmd", "{@_chassis_cmd}",
            "Chassis command interface used to stop the robot"),
    };
}

BT::NodeStatus StopChassisMotion::tick() {
    IChassisCommand* cmd = nullptr;
    if (!getInput("chassis_cmd", cmd) || !cmd) return BT::NodeStatus::FAILURE;
    cmd->stop();
    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::StopChassisMotion>("StopChassisMotion");
}
