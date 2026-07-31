// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace finenav::localizer {

/// OdometryCache — latest odometry reading.
///
/// Pure cache, no integration formula.  Each incoming odometry message
/// simply overwrites the stored value.
struct OdometryCache {
    void apply(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
        position_    = position;
        orientation_ = orientation;
        has_data_    = true;
    }

    [[nodiscard]] bool              hasData()     const { return has_data_; }
    [[nodiscard]] const Eigen::Vector3d&    position()    const { return position_; }
    [[nodiscard]] const Eigen::Quaterniond& orientation() const { return orientation_; }

private:
    Eigen::Vector3d    position_{0, 0, 0};
    Eigen::Quaterniond orientation_{1, 0, 0, 0};
    bool has_data_{false};
};

/// Compose the world anchor with the latest odometry → world-frame robot state.
///
///     state = anchor ⊕ odometry
///
/// where ⊕ denotes rigid composition:  p = a_p + a_q * o_p
///                                      q = a_q * o_q
inline void composeState(const Eigen::Vector3d&    anchor_position,
                         const Eigen::Quaterniond& anchor_orientation,
                         const Eigen::Vector3d&    odom_position,
                         const Eigen::Quaterniond& odom_orientation,
                         Eigen::Vector3d&          out_position,
                         Eigen::Quaterniond&       out_orientation) {
    out_position    = anchor_position + anchor_orientation * odom_position;
    out_orientation = anchor_orientation * odom_orientation;
}

}  // namespace finenav::localizer
