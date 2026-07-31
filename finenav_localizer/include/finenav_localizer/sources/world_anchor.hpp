// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace finenav::localizer {

/// WorldAnchor — the odometry frame's placement in the world frame.
///
/// An absolute-pose observation tells us where the robot is in the world.
/// Given the latest odometry reading at that same instant we can back out
/// where the odometry frame itself sits inside the world.  That "anchor"
/// is then held steady while odometry continues to stream in — composing
/// the anchor with each new odometry reading gives the current robot pose.
///
///     robot_state = anchor ⊕ odometry
///
/// When smoothing is enabled (pose_smoothing_time > 0), the anchor moves
/// toward each new target via EMA rather than jumping instantly.
class WorldAnchor {
public:
    WorldAnchor() = default;

    /// @param smoothing_time_sec  0.0 → instant (alpha = 1.0)
    /// @param tick_rate_hz        Internal timer frequency
    void configure(double smoothing_time_sec, double tick_rate_hz);

    /// Directly set the anchor (instant correction, smoothing = 0).
    void set(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

    /// Queue a target anchor for EMA convergence (smoothing > 0).
    void setTarget(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

    /// Advance the EMA by one tick.
    /// @param[out] position    Updated anchor position (only valid when true).
    /// @param[out] orientation Updated anchor orientation (only valid when true).
    /// @return true if the anchor was modified this tick.
    bool tick(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

    /// @name Status / accessors
    /// @{
    [[nodiscard]] bool isReady()      const { return ready_; }
    [[nodiscard]] bool hasPoseData()  const { return has_pose_data_; }
    [[nodiscard]] const Eigen::Vector3d&    position()    const { return position_; }
    [[nodiscard]] const Eigen::Quaterniond& orientation() const { return orientation_; }
    /// @}

private:
    void freezeDrift();

    double alpha_{1.0};
    bool   ready_{false};
    bool   has_pose_data_{false};

    Eigen::Vector3d    position_{0, 0, 0};
    Eigen::Quaterniond orientation_{1, 0, 0, 0};

    // ── EMA internals ──────────────────────────────────────────────────────
    struct Target { Eigen::Vector3d p; Eigen::Quaterniond q; };
    std::optional<Target> target_;
    Eigen::Vector3d    drift_p_{0, 0, 0};
    Eigen::Quaterniond drift_q_{1, 0, 0, 0};
    bool has_drift_{false};
};

}  // namespace finenav::localizer
