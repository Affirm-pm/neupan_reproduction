// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_interfaces/msg/trajectory.hpp"

namespace finenav::navigator {

class IChassisCommand {
public:
    virtual ~IChassisCommand() = default;
    virtual void publish(const finenav_interfaces::msg::Trajectory& traj) = 0;
};

}  // namespace finenav::navigator
