// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_interfaces/msg/trajectory.hpp"

namespace finenav::navigator {

class IChassisCommand {
public:
    virtual ~IChassisCommand() = default;
    virtual bool validate(const finenav_interfaces::msg::Trajectory& traj) const = 0;
    virtual void publish(const finenav_interfaces::msg::Trajectory& traj) = 0;
    virtual void stop() = 0;
};

}  // namespace finenav::navigator
