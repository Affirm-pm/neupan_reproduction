// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <array>
#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "finenav_localizer/observation_queue.hpp"

namespace finenav::localizer {

// ── Tagged wrappers carrying per-sensor config alongside the ROS message ────────

/// Per-sensor overrides carried alongside a pose observation.
struct TaggedPoseObs {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    int                  smooth{1};
    double               gate_dist{1e9};
    std::array<bool, 6>  mask{{true, true, true, true, true, true}};
    std::array<double, 6> cov_override{};  // < 0 → use from msg; ≥ 0 → override
};

/// Per-sensor overrides carried alongside a twist observation.
struct TaggedTwistObs {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    int                  smooth{1};
    double               gate_dist{1e9};
    std::array<bool, 6>  mask{{true, true, true, true, true, true}};
    std::array<double, 6> cov_override{};  // < 0 → use from msg; ≥ 0 → override
};

// ── Relative-pose observation (two-frame odometry displacement) ─────────────────

/// A body-frame displacement between two consecutive odometry frames.
struct RelativePoseObs {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d             delta_p_body_obs;       // t_{k-1} → t_k
    Sophus::SO3d                delta_R_obs;            // R_{k-1} → R_k
    Eigen::Matrix<double, 6, 6> R_noise;                // combined covariance (6×6)
    rclcpp::Time                stamp_to;
    rclcpp::Time                stamp_from;

    int                  smooth{1};
    double               gate_dist{1e9};
    std::array<bool, 6>  mask{{true, true, true, true, true, true}};
};

// ── Queue aliases ──────────────────────────────────────────────────────────────

using PoseObsQueue        = AgedObservationQueue<TaggedPoseObs>;
using TwistObsQueue       = AgedObservationQueue<TaggedTwistObs>;
using RelativePoseObsQueue = AgedObservationQueue<RelativePoseObs>;

}  // namespace finenav::localizer
