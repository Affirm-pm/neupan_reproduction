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
    void acceptTwist(const Eigen::Vector3d& v_body,
                     const Eigen::Vector3d& omega_body,
                     const rclcpp::Time& stamp,
                     finenav::core::RobotState& state);

    /// Compute state = ref_map ⊕ (current_odom ⊖ ref_odom).
    /// On first call or after resetAnchor(), captures the current odom
    /// frame as the new reference (no state change).
    void acceptRelPose(const nav_msgs::msg::Odometry& msg,
                       finenav::core::RobotState& state);

    /// Update the map-frame reference point.  Next acceptRelPose call
    /// will snapshot the odom frame at that moment as ref_odom.
    void resetAnchor(const finenav::core::RobotState& map_state);

private:
    // ── Twist-mode state ─────────────────────────────────────────────────
    struct TwistRef {
        Eigen::Vector3d v_body;
        Eigen::Vector3d omega_body;
        rclcpp::Time    stamp;
    };
    std::optional<TwistRef> last_twist_;
    bool has_twist_source_{false};

    // ── RelPose reference frame ──────────────────────────────────────────
    Eigen::Vector3d    ref_map_p_{0, 0, 0};
    Eigen::Quaterniond ref_map_q_{1, 0, 0, 0};
    Eigen::Vector3d    ref_odom_p_{0, 0, 0};
    Eigen::Quaterniond ref_odom_q_{1, 0, 0, 0};
    bool ref_ready_{false};  ///< true once ref_odom has been captured
    rclcpp::Time prev_stamp_; ///< reject out-of-order
};

}  // namespace finenav::localizer
