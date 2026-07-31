// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/sources/integrator.hpp"

namespace finenav::localizer {

namespace {

inline Eigen::Quaterniond expOmegaDt(const Eigen::Vector3d& omega, double dt) {
    const double angle = omega.norm() * dt;
    if (angle < 1e-12) return Eigen::Quaterniond::Identity();
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, omega.normalized()));
}

}  // namespace

void Integrator::acceptTwist(const Eigen::Vector3d& v_body,
                              const Eigen::Vector3d& omega_body,
                              const rclcpp::Time& stamp,
                              finenav::core::RobotState& state) {
    if (last_twist_.has_value()) {
        const double dt = (stamp - last_twist_->stamp).seconds();
        if (dt > 0.0) {
            state.position +=
                state.orientation * (last_twist_->v_body * dt);
            state.orientation =
                state.orientation * expOmegaDt(last_twist_->omega_body, dt);
        }
    }

    // Reject out-of-order or duplicate
    if (last_twist_.has_value() && stamp <= last_twist_->stamp) return;

    last_twist_ = TwistRef{v_body, omega_body, stamp};
    has_twist_source_ = true;
}

void Integrator::resetAnchor(const finenav::core::RobotState& map_state) {
    ref_map_p_ = map_state.position;
    ref_map_q_ = map_state.orientation;
    ref_ready_ = false;  // force re-capture of ref_odom on next msg
}

void Integrator::acceptRelPose(const nav_msgs::msg::Odometry& msg,
                                finenav::core::RobotState& state) {
    const rclcpp::Time stamp(msg.header.stamp, RCL_ROS_TIME);

    // Reject out-of-order
    if (ref_ready_ && stamp <= prev_stamp_) return;
    prev_stamp_ = stamp;

    const Eigen::Quaterniond q_curr(
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    const Eigen::Vector3d p_curr(
        msg.pose.pose.position.x, msg.pose.pose.position.y,
        msg.pose.pose.position.z);

    if (!ref_ready_) {
        // (Re)capture odometry reference at this frame — no state change
        ref_odom_p_ = p_curr;
        ref_odom_q_ = q_curr;
        ref_ready_  = true;

        // On the very first odometry frame (before any external anchor),
        // bootstrap ref_map to match so that the subsequent formula
        //   state = ref_map ∘ (ref_odom⁻¹ ∘ odom_curr)
        // reproduces the absolute odometry pose.
        if (!odom_anchored_) {
            ref_map_p_ = p_curr;
            ref_map_q_ = q_curr;
            odom_anchored_ = true;
        }
        return;
    }

    // delta = ref_odom⁻¹ ∘ current_odom  (in odometry frame)
    const Eigen::Vector3d    dp = ref_odom_q_.conjugate() * (p_curr - ref_odom_p_);
    const Eigen::Quaterniond dq = ref_odom_q_.conjugate() * q_curr;

    // state = ref_map ∘ delta  →  place odom delta into the anchored map frame
    state.position    = ref_map_p_ + ref_map_q_ * dp;
    state.orientation = ref_map_q_ * dq;
}

}  // namespace finenav::localizer
