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

/// Velocity estimation.
///
/// Three modes, chosen at construction:
///   None     — no velocity source; state velocity is never touched.
///   Explicit — velocity is set directly from an external source
///              (twist message or odometry twist).
///   AutoDiff — velocity is computed from consecutive absolute-pose
///              observations via finite difference.

class VelocityProvider {
public:
    /// Overwrite velocity on @p state from a twist source.
    void setFromTwist(const Eigen::Vector3d& v_body,
                      const Eigen::Vector3d& omega_body,
                      finenav::core::RobotState& state) const;

    /// Finite-difference velocity from consecutive absolute-pose observations.
    void autoDiffFromPose(const Eigen::Vector3d& p,
                          const Eigen::Quaterniond& q,
                          const rclcpp::Time& stamp,
                          finenav::core::RobotState& state);

private:

    // ── AutoDiff state ───────────────────────────────────────────────────
    bool has_prev_{false};
    rclcpp::Time       prev_stamp_{0, 0, RCL_ROS_TIME};  // Explicitly use RCL_ROS_TIME to prevent time source mismatch
    Eigen::Vector3d    prev_p_{0, 0, 0};
    Eigen::Quaterniond prev_q_{1, 0, 0, 0};
};

}  // namespace finenav::localizer
