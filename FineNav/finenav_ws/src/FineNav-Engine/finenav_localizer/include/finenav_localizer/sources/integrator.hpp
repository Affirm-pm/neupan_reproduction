// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>

#include "finenav_core/robot_state/robot_state.hpp"

namespace finenav::localizer {

/// Dead-reckoning integrator — twist integration only.
///
/// Integrates body-frame velocity references over time.
/// Odometry-frame pose integration is handled by the WorldAnchor +
/// OdometryCache architecture (see world_anchor.hpp, odometry_cache.hpp).
class Integrator {
public:
    void acceptTwist(const Eigen::Vector3d& v_body,
                     const Eigen::Vector3d& omega_body,
                     const rclcpp::Time& stamp,
                     finenav::core::RobotState& state);

private:
    struct TwistRef {
        Eigen::Vector3d v_body;
        Eigen::Vector3d omega_body;
        rclcpp::Time    stamp{0, 0, RCL_ROS_TIME};  // Explicitly use RCL_ROS_TIME to prevent time source mismatch
    };
    std::optional<TwistRef> last_twist_;
};

}  // namespace finenav::localizer
