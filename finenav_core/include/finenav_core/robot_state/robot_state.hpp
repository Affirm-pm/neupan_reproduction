// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace finenav {

/**
 *  @brief RobotState — the canonical robot state representation in FineNav-Engine.
 */
struct RobotState {
    Eigen::Vector3d    position;           ///< world-frame position (m)
    Eigen::Quaterniond orientation;        ///< world ← body rotation
    Eigen::Vector3d    linear_velocity;    ///< body-frame linear velocity (m/s)
    Eigen::Vector3d    angular_velocity;   ///< body-frame angular velocity (rad/s)
};


/**
 * @brief IRobotStateProvider — abstract interface for robot state queries.
 * @details  The pattern is pull-based：consumers call getRobotState() when they need the state,
 */
class IRobotStateProvider {
public:
    virtual ~IRobotStateProvider() = default;

    /// @brief Return the latest available robot state.
    /// @note  Must be thread-safe.  Implementations typically hold a shared_mutex
    ///        on a cached copy updated by an internal timer thread.
    virtual RobotState getRobotState() const = 0;
};

}  // namespace finenav
