// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/time.hpp>

namespace finenav::core {

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
 * @details  The pattern is pull-based: consumers call getRobotState() when they need the state.
 */
class IRobotStateProvider {
public:
    virtual ~IRobotStateProvider() = default;

    /**
     * @brief Return the latest available robot state.
     * @return Optional RobotState; std::nullopt if no state is available.
     * @note  Must be thread-safe.
     */
    virtual std::optional<RobotState> getRobotState() const = 0;

    /**
     * @brief Return the latest available robot state.
     * @param t Query timestamp.
     * @return Optional RobotState; std::nullopt if no state is available.
     * @note  Must be thread-safe.
     */
    virtual std::optional<RobotState> getRobotState(const rclcpp::Time& t) const = 0;
};

}  // namespace finenav::core
