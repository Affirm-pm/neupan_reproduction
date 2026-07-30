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

void Integrator::acceptRelPose(const nav_msgs::msg::Odometry& msg,
                                finenav::core::RobotState& state) {
    // Reject out-of-order
    if (last_odom_.has_value() &&
        rclcpp::Time(msg.header.stamp, RCL_ROS_TIME) <=
        rclcpp::Time(last_odom_->header.stamp, RCL_ROS_TIME)) {
        return;
    }

    if (last_odom_.has_value()) {
        const auto& prev = *last_odom_;
        const auto& curr = msg;

        const Eigen::Quaterniond q_prev(
            prev.pose.pose.orientation.w, prev.pose.pose.orientation.x,
            prev.pose.pose.orientation.y, prev.pose.pose.orientation.z);
        const Eigen::Quaterniond q_curr(
            curr.pose.pose.orientation.w, curr.pose.pose.orientation.x,
            curr.pose.pose.orientation.y, curr.pose.pose.orientation.z);

        const Eigen::Vector3d p_prev(prev.pose.pose.position.x,
                                     prev.pose.pose.position.y,
                                     prev.pose.pose.position.z);
        const Eigen::Vector3d p_curr(curr.pose.pose.position.x,
                                     curr.pose.pose.position.y,
                                     curr.pose.pose.position.z);

        const Eigen::Vector3d    dp = q_prev.conjugate() * (p_curr - p_prev);
        const Eigen::Quaterniond dq = q_prev.conjugate() * q_curr;

        state.position += state.orientation * dp;
        state.orientation = state.orientation * dq;
    }
    last_odom_ = msg;
}

}  // namespace finenav::localizer
