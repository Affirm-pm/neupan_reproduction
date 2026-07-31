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
}

}  // namespace finenav::localizer
