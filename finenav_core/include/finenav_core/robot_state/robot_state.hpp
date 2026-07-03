// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace finenav {

// =============================================================================
// RobotState — the canonical robot state representation for framework consumers.
//
// All quantities use Eigen types (no ROS message dependency).  Velocity is
// expressed in the **body frame**, consistent with FineNavLocalizer::getState().
// =============================================================================

struct RobotState {
    Eigen::Vector3d    position;           ///< world-frame position (m)
    Eigen::Quaterniond orientation;        ///< world ← body rotation
    Eigen::Vector3d    linear_velocity;    ///< body-frame linear velocity (m/s)
    Eigen::Vector3d    angular_velocity;   ///< body-frame angular velocity (rad/s)
};

// =============================================================================
// IRobotStateProvider — abstract interface for robot state queries.
//
// This is the single, framework-wide contract for obtaining the current robot
// state.  finenav_localizer::FineNavLocalizer implements it.  Framework
// components (MapServer, ControlLayer, Navigator) consume it through this
// interface so that finenav_core does not depend on finenav_localizer.
//
// The pattern is pull-based (service-style): consumers call getRobotState()
// when they need the state, typically at the update rate of 10-50 Hz.  This is
// NOT a streaming / message-passing interface.
// =============================================================================

class IRobotStateProvider {
public:
    virtual ~IRobotStateProvider() = default;

    /// @brief Return the latest available robot state.
    /// @note  Must be thread-safe.  Implementations typically hold a shared_mutex
    ///        on a cached copy updated by an internal timer thread.
    virtual RobotState getRobotState() const = 0;
};

}  // namespace finenav
