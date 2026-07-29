// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>

#include "finenav_core/robot_state/robot_state.hpp"

namespace finenav::localizer {

/// A single-step position + orientation correction produced by the smoothing tick.
struct PoseCorrection {
    Eigen::Vector3d    position_delta{0, 0, 0};       ///< world-frame, additive
    Eigen::Quaterniond orientation_delta{1, 0, 0, 0}; ///< body-frame, right-multiply
};

/// Absolute-pose anchor with EMA smoothing.
///
/// feed() stores an absolute-pose observation as the active convergence target.
/// tick() is called by the internal timer — it returns a PoseCorrection
/// ("virtual observation") while converging toward the target, or nullopt when
/// the target has been reached / no target is active.
///
/// Convergence speed is set via configure(), which derives a per-tick alpha
/// from a user-facing smoothing-time parameter.

class PoseAnchor {
public:
    PoseAnchor() = default;

    /// @param smoothing_time_sec  0.0 = instant (alpha = 1.0)
    /// @param tick_rate_hz        Fixed internal timer frequency
    void configure(double smoothing_time_sec, double tick_rate_hz);

    /// Ingest a new absolute-pose observation.  Replaces any active target.
    void feed(const Eigen::Vector3d& p, const Eigen::Quaterniond& q,
              const rclcpp::Time& stamp);

    /// Advance the smoothing by one tick.
    /// @return Correction to apply, or nullopt if nothing to do this tick.
    std::optional<PoseCorrection> tick(const finenav::core::RobotState& current_state);

    /// @name Status / accessors
    /// @{
    [[nodiscard]] bool hasAbsolutePoseData() const { return has_pose_data_; }
    [[nodiscard]] bool hasReceivedPose()      const { return last_pose_stamp_.has_value(); }

    [[nodiscard]] const Eigen::Vector3d&    lastPosePosition()    const { return last_pose_p_; }
    [[nodiscard]] const Eigen::Quaterniond& lastPoseOrientation() const { return last_pose_q_; }
    [[nodiscard]] const rclcpp::Time&       lastPoseStamp()       const { return *last_pose_stamp_; }
    /// @}

private:
    void freezeDrift(const finenav::core::RobotState& current_state);
    [[nodiscard]] bool driftIsNearZero() const;

    double alpha_{1.0};

    struct Target { Eigen::Vector3d p; Eigen::Quaterniond q; };
    std::optional<Target> pending_target_;

    Eigen::Vector3d    remaining_drift_p_{0, 0, 0};
    Eigen::Quaterniond remaining_drift_q_{1, 0, 0, 0};
    bool has_drift_{false};

    bool                        has_pose_data_{false};
    std::optional<rclcpp::Time> last_pose_stamp_;
    Eigen::Vector3d             last_pose_p_{0, 0, 0};
    Eigen::Quaterniond          last_pose_q_{1, 0, 0, 0};
};

}  // namespace finenav::localizer
