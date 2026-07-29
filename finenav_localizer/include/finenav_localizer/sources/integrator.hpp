// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include "finenav_core/robot_state/robot_state.hpp"

namespace finenav::localizer {

/// Dead-reckoning position integrator.
///
/// Two mutually exclusive modes (set at construction):
///   Twist   — integrate body-frame velocity references over dt.
///   RelPose — apply frame-to-frame odometry pose deltas.
///
/// In both modes each observation is applied immediately on arrival;
/// there is no timer involvement.

class Integrator {
public:
    /// Integrate body-frame velocity from the previous reference to @p stamp,
    /// then store this twist as the new reference.
    void acceptTwist(const Eigen::Vector3d& v_body,
                     const Eigen::Vector3d& omega_body,
                     const rclcpp::Time& stamp,
                     finenav::core::RobotState& state);

    /// Diff consecutive Odometry pose fields, apply the body-frame delta.
    void acceptRelPose(const nav_msgs::msg::Odometry& msg,
                       finenav::core::RobotState& state);

private:

    // ── Twist-mode state ─────────────────────────────────────────────────
    struct TwistRef {
        Eigen::Vector3d v_body;
        Eigen::Vector3d omega_body;
        rclcpp::Time    stamp;
    };
    std::optional<TwistRef> last_twist_;
    bool has_twist_source_{false};

    // ── RelPose-mode state ───────────────────────────────────────────────
    std::optional<nav_msgs::msg::Odometry> last_odom_;
};

}  // namespace finenav::localizer
