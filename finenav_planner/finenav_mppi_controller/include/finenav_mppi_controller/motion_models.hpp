// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
#define NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "finenav_mppi_controller/models/control_sequence.hpp"
#include "finenav_mppi_controller/models/constraints.hpp"
#include "finenav_mppi_controller/models/state.hpp"

//#include "finenav_mppi_controller/tools/parameters_handler.hpp"

namespace mppi
{

/**
 * @class mppi::MotionModel
 * @brief Abstract motion model for modeling a vehicle
 */
class MotionModel
{
public:
  /**
    * @brief Constructor for mppi::MotionModel
    */
  MotionModel() = default;

  /**
    * @brief Destructor for mppi::MotionModel
    */
  virtual ~MotionModel() = default;

  void setConstraints(const models::ControlConstraints & constraints, float model_dt)
  {
    constraints_ = constraints;
    model_dt_ = model_dt;
    constraints_configured_ = true;
  }

  /**
   * @brief With input velocities, find the vehicle's output velocities
   * @param state Contains control velocities to use to populate vehicle velocities
   */
  virtual void predict(models::State & state)
  {
    if (state.vx.cols() == 0) {
      return;
    }

    if (!constraints_configured_) {
      state.vx = state.cvx;
      state.wz = state.cwz;
      if (isHolonomic()) {
        state.vy = state.cvy;
      } else {
        state.vy.setZero();
      }
      return;
    }

    // state.v(:, 0) contains feedback only as the projection anchor. Every
    // output column represents the interval driven by the control at t.
    for (Eigen::Index row = 0; row < state.vx.rows(); ++row) {
      float previous_vx = state.vx(row, 0);
      float previous_vy = state.vy(row, 0);
      float previous_wz = state.wz(row, 0);

      for (Eigen::Index t = 0; t < state.vx.cols(); ++t) {
        const Eigen::Index control_index = t;
        const float vx = projectVelocity(
          state.cvx(row, control_index), previous_vx,
          constraints_.vx_min, constraints_.vx_max,
          constraints_.ax_min, constraints_.ax_max, model_dt_);
        const float vy = isHolonomic() ? projectVelocity(
          state.cvy(row, control_index), previous_vy,
          -constraints_.vy, constraints_.vy,
          constraints_.ay_min, constraints_.ay_max, model_dt_) : 0.0F;
        const float wz = projectAngularVelocity(
          state.cwz(row, control_index), previous_wz, vx, model_dt_);

        state.vx(row, t) = vx;
        state.vy(row, t) = vy;
        state.wz(row, t) = wz;
        previous_vx = vx;
        previous_vy = vy;
        previous_wz = wz;
      }
    }
  }

  static float projectVelocity(
    float requested, float previous, float velocity_min, float velocity_max,
    float acceleration_min, float acceleration_max, float dt)
  {
    const bool reverse = std::abs(previous) > 1e-6F ?
      previous < 0.0F : requested < 0.0F;
    const float acceleration_lower = reverse ? -acceleration_max : acceleration_min;
    const float acceleration_upper = reverse ? -acceleration_min : acceleration_max;
    const float reachable_lower = previous + acceleration_lower * dt;
    const float reachable_upper = previous + acceleration_upper * dt;
    const float lower = std::max(
      velocity_min, reachable_lower);
    const float upper = std::min(
      velocity_max, reachable_upper);

    // If feedback is already outside the configured velocity envelope, the
    // absolute and acceleration intervals can be disjoint. Preserve the
    // physically reachable interval instead of creating an instantaneous
    // velocity jump by clamping feedback before applying acceleration limits.
    if (lower > upper) {
      return std::clamp(requested, reachable_lower, reachable_upper);
    }

    return std::clamp(requested, lower, upper);
  }

  virtual float projectAngularVelocity(float requested, float previous, float, float dt) const
  {
    const float reachable_lower = previous - constraints_.az_max * dt;
    const float reachable_upper = previous + constraints_.az_max * dt;
    const float lower = std::max(-constraints_.wz, reachable_lower);
    const float upper = std::min(constraints_.wz, reachable_upper);
    if (lower > upper) {
      return std::clamp(requested, reachable_lower, reachable_upper);
    }

    return std::clamp(requested, lower, upper);
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  virtual bool isHolonomic() = 0;

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  virtual void applyConstraints(models::ControlSequence & /*control_sequence*/) {}

protected:
  models::ControlConstraints constraints_{};
  float model_dt_{0.0F};
  bool constraints_configured_{false};
};

/**
 * @class mppi::AckermannMotionModel
 * @brief Ackermann motion model
 */
class AckermannMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::AckermannMotionModel
    */
  AckermannMotionModel()
  {
    min_turning_r_= 0.2 ;
  }

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }

  /**
   * @brief Apply hard vehicle constraints to a control sequence
   * @param control_sequence Control sequence to apply constraints to
   */
  void applyConstraints(models::ControlSequence & control_sequence) override
  {
    auto & vx = control_sequence.vx;
    auto & wz = control_sequence.wz;
    for (Eigen::Index i = 0; i < vx.size(); ++i) {
      const float turning_limit = std::abs(vx(i)) / min_turning_r_;
      if (std::abs(wz(i)) > turning_limit) {
        wz(i) = std::copysign(turning_limit, wz(i));
      }
    }
  }

  float projectAngularVelocity(float requested, float previous, float vx, float dt) const override
  {
    const float turning_limit = std::abs(vx) / min_turning_r_;
    const float kinematic_lower = -turning_limit;
    const float kinematic_upper = turning_limit;

    const float reachable_lower = previous - constraints_.az_max * dt;
    const float reachable_upper = previous + constraints_.az_max * dt;
    const float lower = std::max(
      std::max(-constraints_.wz, reachable_lower), kinematic_lower);
    const float upper = std::min(
      std::min(constraints_.wz, reachable_upper), kinematic_upper);
    if (lower <= upper) {
      return std::clamp(requested, lower, upper);
    }

    // Preserve acceleration reachability if the absolute yaw and acceleration
    // intervals are disjoint because feedback is already out of range.
    const float reachable_kinematic_lower = std::max(reachable_lower, kinematic_lower);
    const float reachable_kinematic_upper = std::min(reachable_upper, kinematic_upper);
    if (reachable_kinematic_lower <= reachable_kinematic_upper) {
      return std::clamp(
        requested, reachable_kinematic_lower, reachable_kinematic_upper);
    }

    // No value can satisfy both the acceleration and turning-radius intervals.
    // Keep the existing kinematic-priority fallback for this infeasible case.
    return std::clamp(previous, kinematic_lower, kinematic_upper);
  }

  /**
   * @brief Get minimum turning radius of ackermann drive
   * @return Minimum turning radius
   */
  float getMinTurningRadius() const {return min_turning_r_;}

private:
  float min_turning_r_{0};
};

/**
 * @class mppi::DiffDriveMotionModel
 * @brief Differential drive motion model
 */
class DiffDriveMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::DiffDriveMotionModel
    */
  DiffDriveMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return false;
  }
};

/**
 * @class mppi::OmniMotionModel
 * @brief Omnidirectional motion model
 */
class OmniMotionModel : public MotionModel
{
public:
  /**
    * @brief Constructor for mppi::OmniMotionModel
    */
  OmniMotionModel() = default;

  /**
   * @brief Whether the motion model is holonomic, using Y axis
   * @return Bool If holonomic
   */
  bool isHolonomic() override
  {
    return true;
  }
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__MOTION_MODELS_HPP_
