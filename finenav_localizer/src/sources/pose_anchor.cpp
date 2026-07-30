// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/sources/pose_anchor.hpp"

#include <cmath>

namespace finenav::localizer {

void PoseAnchor::configure(double smoothing_time_sec, double tick_rate_hz) {
    if (smoothing_time_sec <= 0.0) {
        alpha_ = 1.0;
    } else {
        const double N = smoothing_time_sec * tick_rate_hz;
        // EMA: after N ticks, remaining drift = (1-α)^N · initial
        // We want (1-α)^N = 0.05  →  α = 1 − 0.05^(1/N)
        // i.e. 95 % converged within smoothing_time_sec
        alpha_ = 1.0 - std::pow(0.05, 1.0 / N);
    }
}

void PoseAnchor::feed(const Eigen::Vector3d& p, const Eigen::Quaterniond& q,
                       const rclcpp::Time& stamp) {
    last_pose_p_     = p;
    last_pose_q_     = q;
    last_pose_stamp_ = stamp;
    pending_target_  = Target{p, q};
    has_pose_data_   = true;
}

std::optional<PoseCorrection> PoseAnchor::tick(
    const finenav::core::RobotState& current_state) {

    if (pending_target_.has_value()) {
        freezeDrift(current_state);
    }

    if (!has_drift_ || driftIsNearZero()) {
        has_drift_ = false;
        return std::nullopt;
    }

    PoseCorrection corr;
    corr.position_delta    = alpha_ * remaining_drift_p_;
    corr.orientation_delta =
        Eigen::Quaterniond::Identity().slerp(alpha_, remaining_drift_q_);

    remaining_drift_p_ *= (1.0 - alpha_);
    remaining_drift_q_  = Eigen::Quaterniond::Identity().slerp(
        1.0 - alpha_, remaining_drift_q_);

    return corr;
}

void PoseAnchor::freezeDrift(const finenav::core::RobotState& current_state) {
    remaining_drift_p_ = pending_target_->p - current_state.position;
    remaining_drift_q_ =
        current_state.orientation.conjugate() * pending_target_->q;
    pending_target_.reset();
    has_drift_ = true;
}

bool PoseAnchor::driftIsNearZero() const {
    constexpr double kEpsPos = 1e-6;
    constexpr double kEpsRot = 1e-9;
    return remaining_drift_p_.squaredNorm() <= kEpsPos * kEpsPos &&
           Eigen::Quaterniond::Identity().angularDistance(remaining_drift_q_) < kEpsRot;
}

}  // namespace finenav::localizer
