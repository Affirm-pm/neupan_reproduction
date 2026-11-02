// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/sources/velocity_provider.hpp"

#include "finenav_core/robot_state/eigen_conversions.hpp"

namespace finenav::localizer {

void VelocityProvider::setFromTwist(const Eigen::Vector3d& v_body,
                                     const Eigen::Vector3d& omega_body,
                                     finenav::core::RobotState& state) const {
    core::toMsgLinearVel(v_body, state);
    core::toMsgAngularVel(omega_body, state);
}

void VelocityProvider::autoDiffFromPose(const Eigen::Vector3d& p,
                                         const Eigen::Quaterniond& q,
                                         const rclcpp::Time& stamp,
                                         finenav::core::RobotState& state) {
    const rclcpp::Time stamp_ros(stamp.nanoseconds(), RCL_ROS_TIME);
    if (has_prev_) {
        const double dt = (stamp_ros - prev_stamp_).seconds();
        if (dt > 1e-6) {
            Eigen::Vector3d lin_vel = prev_q_.conjugate() * (p - prev_p_) / dt;
            core::toMsgLinearVel(lin_vel, state);

            const Eigen::Quaterniond dq = prev_q_.conjugate() * q;
            const Eigen::AngleAxisd aa(dq);
            Eigen::Vector3d ang_vel = aa.axis() * aa.angle() / dt;
            core::toMsgAngularVel(ang_vel, state);
        }
    }
    prev_p_      = p;
    prev_q_      = q;
    prev_stamp_  = stamp_ros;
    has_prev_    = true;
}

}  // namespace finenav::localizer
