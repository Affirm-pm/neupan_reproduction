// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/sources/velocity_provider.hpp"

namespace finenav::localizer {

void VelocityProvider::setFromTwist(const Eigen::Vector3d& v_body,
                                     const Eigen::Vector3d& omega_body,
                                     finenav::core::RobotState& state) const {
    state.linear_velocity  = v_body;
    state.angular_velocity = omega_body;
}

void VelocityProvider::autoDiffFromPose(const Eigen::Vector3d& p,
                                         const Eigen::Quaterniond& q,
                                         const rclcpp::Time& stamp,
                                         finenav::core::RobotState& state) {
    const rclcpp::Time stamp_ros(stamp.nanoseconds(), RCL_ROS_TIME);
    if (has_prev_) {
        const double dt = (stamp_ros - prev_stamp_).seconds();
        if (dt > 1e-6) {
            state.linear_velocity =
                prev_q_.conjugate() * (p - prev_p_) / dt;
            const Eigen::Quaterniond dq = prev_q_.conjugate() * q;
            const Eigen::AngleAxisd aa(dq);
            state.angular_velocity = aa.axis() * aa.angle() / dt;
        }
    }
    prev_p_      = p;
    prev_q_      = q;
    prev_stamp_  = stamp_ros;
    has_prev_    = true;
}

}  // namespace finenav::localizer
