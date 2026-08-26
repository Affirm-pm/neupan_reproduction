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

#ifndef NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
#define NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_

#pragma once

#include <string>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "finenav_mppi_controller/models/optimizer_settings.hpp"
#include "finenav_mppi_controller/motion_models.hpp"
#include "finenav_mppi_controller/critic_manager.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/models/trajectories.hpp"
#include "finenav_mppi_controller/models/path.hpp"
#include "finenav_mppi_controller/tools/noise_generator.hpp"
//#include "finenav_mppi_controller/tools/parameters_handler.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"



namespace mppi
{

struct OptimizationResult
{
  models::BatchArray controls;
  models::BatchArray trajectory;
};

/**
 * @class mppi::Optimizer
 * @brief Main algorithm optimizer of the MPPI Controller
 */


class Optimizer
{
public:
  /**
    * @brief Constructor for mppi::Optimizer
    */
  	Optimizer() = default;

  /**
   * @brief Destructor for mppi::Optimizer
   */
  	~Optimizer() {shutdown();}


  /**
   * @brief Initializes optimizer on startup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param dynamic_parameter_handler Parameter handler object
   */
    void initialize(
        const std::string & name,
        std::vector<std::unique_ptr<critics::CriticFunction>> & critics,
        const finenav_mppi_controller::Params & params)
  	{
  	    name_ = name;
  	    getParams(params);

        critics_data_.critic_cost_breakdown = params.visualize ? &critic_cost_breakdown_ : nullptr;

		critic_manager_.on_configure(critics, params);
        noise_generator_.initialize(settings_, isHolonomic(), regenerate_noises_);

  	    reset();

  		std::cout<<"[nav2_mppi_controller] Optimizing initialized"<<std::endl;
  	}

  /**
   * @brief Shutdown for optimizer at process end
   */
    void shutdown()
  	{
  	    noise_generator_.shutdown();
  	}

    void getParams(const finenav_mppi_controller::Params & p)
  	{
  	    auto & s = settings_;

  	    s.model_dt        = static_cast<float>(p.model_dt);
  	    s.time_steps      = static_cast<unsigned int>(p.time_steps);
  	    s.batch_size      = static_cast<unsigned int>(p.batch_size);
  	    s.iteration_count = static_cast<unsigned int>(p.iteration_count);
  	    s.temperature     = static_cast<float>(p.temperature);
  	    s.gamma           = static_cast<float>(p.gamma);

        s.constraints.vx_max = p.vx_max;
        s.constraints.vx_min = p.vx_min;
        s.constraints.vy     = p.vy_max;
        s.constraints.wz     = p.wz_max;
        s.constraints.ax_min = static_cast<float>(p.ax_min);
        s.constraints.ax_max = static_cast<float>(p.ax_max);
        s.constraints.ay_min = static_cast<float>(p.ay_min);
        s.constraints.ay_max = static_cast<float>(p.ay_max);
        s.constraints.az_max = static_cast<float>(p.az_max);

        s.sampling_std.vx = p.vx_std;
        s.sampling_std.vy = p.vy_std;
        s.sampling_std.wz = p.wz_std;
        regenerate_noises_ = p.regenerate_noises;

  	    s.retry_attempt_limit = 1;

  	    setMotionModel(p.motion_model);

  		std::cout<<"[nav2_mppi_controller] Optimizing param prepared"<<std::endl;
  	}

  /**
   * @brief Compute control using MPPI algorithm
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   * @return TwistStamped of the MPPI control
   */
	// geometry_msgs::msg::TwistStamped evalControl(
	OptimizationResult evalControl(
	    const geometry_msgs::msg::PoseStamped & robot_pose,
	    const geometry_msgs::msg::Twist & robot_speed,
	    const nav_msgs::msg::Path & plan,
	    const nav2_mppi_controller::IMapView& map_view)
	{
    prepare(robot_pose, robot_speed, plan);

    do {
      optimize(map_view);
    } while (fallback(critics_data_.fail_flag));
    return finalizeControlSequence();
}


  /**
   * @brief Get the trajectories generated in a cycle for visualization
   * @return Set of trajectories evaluated in cycle
   */
    models::Trajectories & getGeneratedTrajectories()
  	{
  	    return generated_trajectories_;
  	}

    const CriticCostBreakdown & getCriticCostBreakdown() const
    {
        return critic_cost_breakdown_;
    }

  /**
   * @brief Get the optimal trajectory for a cycle for visualization
   * @return Optimal trajectory
   */
    /**
     * @brief Get the optimal control sequence (vx, vy, wz) for each time step
     * @return Eigen array of shape [time_steps, 3] with columns (vx, wz, vy)
     */
    models::BatchArray getControlSequence() const
    {
        models::BatchArray seq(settings_.time_steps, 3);
        seq.col(0) = control_sequence_.vx;
        seq.col(1) = control_sequence_.wz;
        if (isHolonomic()) {
            seq.col(2) = control_sequence_.vy;
        } else {
            seq.col(2).setZero();
        }
        return seq;
    }

    models::BatchArray getOptimizedTrajectory(
      const models::BatchArray & sequence) const
    {
        models::BatchArray trajectories(settings_.time_steps, 3);
        trajectories.setZero();
        integrateStateVelocities(trajectories, sequence);
        return trajectories;
    }

  /**
    * @brief Reset the optimization problem to initial conditions
   */
    void reset()
    {
        state_.reset(settings_.batch_size, settings_.time_steps);
        control_sequence_.reset(settings_.time_steps);
        costs_.resize(settings_.batch_size);
        costs_.setZero();
        generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
        critic_cost_breakdown_.clear();
        noise_generator_.reset(settings_, isHolonomic());
    }


protected:
    OptimizationResult finalizeControlSequence()
    {
        auto controls = getControlSequence();
        auto trajectory = getOptimizedTrajectory(controls);

        shiftControlSequence();

        return {std::move(controls), std::move(trajectory)};
    }

  /**
   * @brief Main function to generate, score, and return trajectories
   */

	void optimize(const nav2_mppi_controller::IMapView& map_view)
	{
	  for (size_t i = 0; i < settings_.iteration_count; ++i) {
        generateNoisedTrajectories();
        critic_manager_.evalTrajectoriesScores(critics_data_, map_view);
        updateControlSequence();
	  }
	}
  /**
   * @brief Prepare state information on new request for trajectory rollouts
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   */
    void prepare(
        const geometry_msgs::msg::PoseStamped & robot_pose,
        const geometry_msgs::msg::Twist & robot_speed,
        const nav_msgs::msg::Path & plan)
	{
	    state_.pose = robot_pose;
	    state_.speed = robot_speed;
	    path_ = utils::toTensor(plan);
	    costs_.fill(0);

	    critics_data_.fail_flag = false;
	    critics_data_.motion_model = motion_model_;
	    critics_data_.furthest_reached_path_point.reset();
	    critics_data_.path_pts_valid.reset();
	}



  /**
   * @brief Set the motion model of the vehicle platform
   * @param model Model string to use
   */
    void setMotionModel(const std::string & model)
    {
        if (model == "DiffDrive") {
            motion_model_ = std::make_shared<DiffDriveMotionModel>();
        } else if (model == "Omni") {
            motion_model_ = std::make_shared<OmniMotionModel>();
        } else if (model == "Ackermann") {
            motion_model_ = std::make_shared<AckermannMotionModel>();
        } else {
            throw std::runtime_error(
                    std::string(
                      "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
                      "or Ackermann"));
        }

        motion_model_->setConstraints(settings_.constraints, settings_.model_dt);
    }

  /**
   * @brief Shift the optimal control sequence after processing for
   * next iterations initial conditions after execution
   */
    void shiftControlSequence()
    {
        const Eigen::Index size = control_sequence_.vx.size();
        if (size > 1) {
          control_sequence_.vx.head(size - 1) = control_sequence_.vx.tail(size - 1);
          control_sequence_.wz.head(size - 1) = control_sequence_.wz.tail(size - 1);
          control_sequence_.vx(size - 1) = control_sequence_.vx(size - 2);
          control_sequence_.wz(size - 1) = control_sequence_.wz(size - 2);

          if (isHolonomic()) {
            control_sequence_.vy.head(size - 1) = control_sequence_.vy.tail(size - 1);
            control_sequence_.vy(size - 1) = control_sequence_.vy(size - 2);
          }
        }
    }

  /**
   * @brief updates generated trajectories with noised trajectories
   * from the last cycle's optimal control
   */
    void generateNoisedTrajectories()
    {
        noise_generator_.setNoisedControls(state_, control_sequence_);
        noise_generator_.generateNextNoises();
        updateStateVelocities(state_);
        integrateStateVelocities(generated_trajectories_, state_);
    }

  /**
   * @brief Apply hard vehicle constraints on control sequence
   */
    void applyControlSequenceConstraints()
    {
        // Apply specialty vehicle constraints before the dynamic projection,
        // matching Nav2's acceleration-aware control-sequence pipeline.
        motion_model_->applyConstraints(control_sequence_);

        float previous_vx = state_.speed.linear.x;
        float previous_vy = state_.speed.linear.y;
        float previous_wz = state_.speed.angular.z;
        const float dt = settings_.model_dt;
        for (Eigen::Index t = 0; t < control_sequence_.vx.size(); ++t) {
            const float vx = MotionModel::projectVelocity(
                control_sequence_.vx(t), previous_vx,
                settings_.constraints.vx_min, settings_.constraints.vx_max,
                settings_.constraints.ax_min, settings_.constraints.ax_max, dt);
            const float vy = isHolonomic() ? MotionModel::projectVelocity(
                control_sequence_.vy(t), previous_vy,
                -settings_.constraints.vy, settings_.constraints.vy,
                settings_.constraints.ay_min, settings_.constraints.ay_max, dt) : 0.0F;
            const float wz = motion_model_->projectAngularVelocity(
                control_sequence_.wz(t), previous_wz, vx, dt);

            control_sequence_.vx(t) = vx;
            control_sequence_.vy(t) = vy;
            control_sequence_.wz(t) = wz;
            previous_vx = vx;
            previous_vy = vy;
            previous_wz = wz;
        }

        motion_model_->applyConstraints(control_sequence_);
    }


  /**
   * @brief  Update velocities in state
   * @param state fill state with velocities on each step
   */
    void updateStateVelocities(
      models::State & state) const
    {
        updateInitialStateVelocities(state);
        propagateStateVelocitiesFromInitials(state);
    }


  /**
   * @brief  Update initial velocity in state
   * @param state fill state
   */
    void updateInitialStateVelocities(
      models::State & state) const
    {
        state.vx.col(0).setConstant(state.speed.linear.x);
        state.wz.col(0).setConstant(state.speed.angular.z);

        if (isHolonomic()) {
            state.vy.col(0).setConstant(state.speed.linear.y);
        } else {
          state.vy.col(0).setZero();
        }
    }
  /**
   * @brief Project each sampled control onto the reachable velocity interval.
   *
   * The measured velocity is the projection anchor before the horizon. It is
   * not stored as an additional rollout knot: state.v(:, t) is the velocity
   * driven by the sampled control at the same t.
   * @param state fill state
   */
    void propagateStateVelocitiesFromInitials(
      models::State & state) const
    {
        motion_model_->predict(state);
    }

  /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
    void integrateStateVelocities(
		models::BatchArray & trajectory,
		const models::BatchArray & sequence) const
    {
        const Eigen::Index time_steps = sequence.rows();
        if (time_steps == 0) {
          return;
        }

        const float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);
        models::Vector yaws(time_steps);
        float last_yaw = initial_yaw;
        for (Eigen::Index t = 0; t < time_steps; ++t) {
          last_yaw += sequence(t, 1) * settings_.model_dt;
          yaws(t) = last_yaw;
        }

        models::Vector yaw_cos(time_steps);
        models::Vector yaw_sin(time_steps);
        yaw_cos(0) = std::cos(initial_yaw);
        yaw_sin(0) = std::sin(initial_yaw);
        if (time_steps > 1) {
          yaw_cos.tail(time_steps - 1) = yaws.head(time_steps - 1).cos();
          yaw_sin.tail(time_steps - 1) = yaws.head(time_steps - 1).sin();
        }

        models::Vector dx = sequence.col(0) * yaw_cos;
        models::Vector dy = sequence.col(0) * yaw_sin;
        if (isHolonomic()) {
          dx -= sequence.col(2) * yaw_sin;
          dy += sequence.col(2) * yaw_cos;
        }

        float x = static_cast<float>(state_.pose.pose.position.x);
        float y = static_cast<float>(state_.pose.pose.position.y);
        for (Eigen::Index t = 0; t < time_steps; ++t) {
          x += dx(t) * settings_.model_dt;
          y += dy(t) * settings_.model_dt;
          trajectory(t, 0) = x;
          trajectory(t, 1) = y;
          trajectory(t, 2) = yaws(t);
        }
    }

	void integrateStateVelocities(
		models::Trajectories & trajectories,
		const models::State & state) const
	{
		integrateBatchStateVelocities(trajectories, state);
	}

    void integrateBatchStateVelocities(
      models::Trajectories & trajectories,
      const models::State & state) const
    {
        const Eigen::Index batch_size = state.vx.rows();
        const Eigen::Index time_steps = state.vx.cols();
        if (batch_size == 0 || time_steps == 0) {
          return;
        }

        const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);
        models::Vector yaws(time_steps);
        models::Vector yaw_cos(time_steps);
        models::Vector yaw_sin(time_steps);
        models::Vector dx(time_steps);
        models::Vector dy(time_steps);

        // RowMajor storage makes each trajectory row contiguous for Eigen operations.
        for (Eigen::Index i = 0; i < batch_size; ++i) {
          const Eigen::Map<const models::Vector> vx_row(
            state.vx.row(i).data(), time_steps);
          const Eigen::Map<const models::Vector> wz_row(
            state.wz.row(i).data(), time_steps);

          float last_yaw = initial_yaw;
          for (Eigen::Index t = 0; t < time_steps; ++t) {
            yaw_cos(t) = std::cos(last_yaw);
            yaw_sin(t) = std::sin(last_yaw);
            last_yaw += wz_row(t) * settings_.model_dt;
            yaws(t) = last_yaw;
          }

          dx = vx_row * yaw_cos;
          dy = vx_row * yaw_sin;
          if (isHolonomic()) {
            const Eigen::Map<const models::Vector> vy_row(
              state.vy.row(i).data(), time_steps);
            dx -= vy_row * yaw_sin;
            dy += vy_row * yaw_cos;
          }

          float x = static_cast<float>(state.pose.pose.position.x);
          float y = static_cast<float>(state.pose.pose.position.y);
          for (Eigen::Index t = 0; t < time_steps; ++t) {
            x += dx(t) * settings_.model_dt;
            y += dy(t) * settings_.model_dt;
            trajectories.x(i, t) = x;
            trajectories.y(i, t) = y;
            trajectories.yaws(i, t) = yaws(t);
          }
        }
    }


  /**
   * @brief Update control sequence with state controls weighted by costs
   * using softmax function
   */
    void updateControlSequence()
    {
        auto & s = settings_;
        auto accumulate_control_cost = [this, &s](
          const models::BatchArray & controls, const models::Vector & sequence, float standard_deviation) {
            models::BatchArray bounded_noises = controls;
            bounded_noises.rowwise() -= sequence.transpose();
            bounded_noises.rowwise() *= sequence.transpose();
            costs_ += s.gamma / std::pow(standard_deviation, 2.0F) * bounded_noises.rowwise().sum();
          };

        accumulate_control_cost(state_.cvx, control_sequence_.vx, s.sampling_std.vx);
        accumulate_control_cost(state_.cwz, control_sequence_.wz, s.sampling_std.wz);

        if (isHolonomic()) {
            accumulate_control_cost(state_.cvy, control_sequence_.vy, s.sampling_std.vy);
        }

        const models::Vector exponents =
          (-(costs_.array() - costs_.minCoeff()) / settings_.temperature).exp();
        const models::Vector softmaxes = exponents / exponents.sum();
        for (Eigen::Index t = 0; t < state_.cvx.cols(); ++t) {
          control_sequence_.vx(t) = (state_.cvx.col(t) * softmaxes).sum();
          control_sequence_.wz(t) = (state_.cwz.col(t) * softmaxes).sum();
          if (isHolonomic()) {
            control_sequence_.vy(t) = (state_.cvy.col(t) * softmaxes).sum();
          }
        }

        applyControlSequenceConstraints();
    }

  /**
   * @brief Whether the motion model is holonomic
   * @return Bool if holonomic to populate `y` axis of state
   */
    bool isHolonomic() const {return motion_model_->isHolonomic();}

  /**
   * @brief Perform fallback behavior to try to recover from a set of trajectories in collision
   * @param fail Whether the system failed to recover from
   */
    bool fallback(bool fail)
    {
        static size_t counter = 0;

        if (!fail) {
            counter = 0;
            return false;
        }

        reset();

        if (++counter > settings_.retry_attempt_limit) {
            counter = 0;
            throw std::runtime_error("Optimizer fail to compute path");
        }

        return true;
    }

protected:
//  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;

  std::shared_ptr<MotionModel> motion_model_;

//  ParametersHandler * parameters_handler_;
  CriticManager critic_manager_;
  NoiseGenerator noise_generator_;

  models::OptimizerSettings settings_;
  bool regenerate_noises_{true};

  models::State state_;
  models::ControlSequence control_sequence_;
  models::Trajectories generated_trajectories_;
  models::Path path_;
  models::Vector costs_;
  CriticCostBreakdown critic_cost_breakdown_;

  CriticData critics_data_ =
  {state_, generated_trajectories_, path_, costs_, settings_.model_dt, false, nullptr,
    std::nullopt, std::nullopt, nullptr};  /// Caution, keep references

};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__OPTIMIZER_HPP_
