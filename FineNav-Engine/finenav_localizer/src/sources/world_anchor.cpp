// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/sources/world_anchor.hpp"

#include <cmath>

namespace finenav::localizer {

void WorldAnchor::configure(double smoothing_time_sec, double tick_rate_hz) {
    if (smoothing_time_sec <= 0.0) {
        alpha_ = 1.0;
    } else {
        const double N = smoothing_time_sec * tick_rate_hz;
        alpha_ = 1.0 - std::pow(0.05, 1.0 / N);
    }
}

void WorldAnchor::set(const Eigen::Vector3d& position,
                      const Eigen::Quaterniond& orientation) {
    position_    = position;
    orientation_ = orientation;
    ready_       = true;
    has_pose_data_ = true;
    // Clear any in-flight EMA state
    target_.reset();
    has_drift_ = false;
}

void WorldAnchor::setTarget(const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation) {
    target_ = Target{position, orientation};
    has_pose_data_ = true;
}

bool WorldAnchor::tick(Eigen::Vector3d& position, Eigen::Quaterniond& orientation) {
    if (target_.has_value()) {
        freezeDrift();
    }

    if (!has_drift_) return false;

    // Check convergence
    constexpr double kEpsPos = 1e-6;
    constexpr double kEpsRot = 1e-9;
    if (drift_p_.squaredNorm() <= kEpsPos &&
        Eigen::Quaterniond::Identity().angularDistance(drift_q_) < kEpsRot) {
        has_drift_ = false;
        return false;
    }

    // Apply one EMA step
    position_    += alpha_ * drift_p_;
    orientation_  = orientation_ * Eigen::Quaterniond::Identity().slerp(alpha_, drift_q_);

    drift_p_ *= (1.0 - alpha_);
    drift_q_  = Eigen::Quaterniond::Identity().slerp(1.0 - alpha_, drift_q_);

    ready_ = true;

    position    = position_;
    orientation = orientation_;
    return true;
}

void WorldAnchor::freezeDrift() {
    drift_p_ = target_->p - position_;
    drift_q_ = orientation_.conjugate() * target_->q;
    target_.reset();
    has_drift_ = true;
}

}  // namespace finenav::localizer
