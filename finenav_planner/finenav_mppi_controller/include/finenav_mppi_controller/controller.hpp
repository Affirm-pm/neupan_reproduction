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

#ifndef NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
#define NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_

#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

//#include "finenav_mppi_controller/tools/path_handler.hpp"
#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"
#include "finenav_mppi_controller/models/constraints.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

#include "rclcpp/rclcpp.hpp"



#include "finenav_core/plan/planner_concept.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"
#include "finenav_mppi_controller/optimizer.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"

namespace nav2_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */


class MPPIController
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
    MPPIController() = default;

    static constexpr finenav::core::IOProfile InputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };
    static constexpr finenav::core::IOProfile OutputProfile = {
        .pose = true,
        .vel  = true,
        .accel= false,
        .time = false
    };

    using IMapView = nav2_mppi_controller::IMapView;
    using Context = finenav::core::PlanningContext<MPPIController>;
    using ConfigType = finenav_mppi_controller::Params;
    void configure(const ConfigType& params) {              //应该传入参数 因测试先去除
        if (params.CostCritic.num_threads <= 0 || params.CostCritic.chunk_size <= 0) {
            throw std::invalid_argument(
                "CostCritic.num_threads and CostCritic.chunk_size must be greater than zero");
        }

        if (params.visualize &&
            (params.visualizer_trajectory_step <= 0 || params.visualizer_time_step <= 0 ||
            params.visualizer_max_trajectories <= 0 || params.visualizer_publish_rate <= 0.0)) {
            throw std::invalid_argument(
                "Visualizer sampling, maximum trajectories, and publish rate must be greater "
                "than zero");
        }

        if (!std::isfinite(params.ax_min) || !std::isfinite(params.ax_max) ||
            !std::isfinite(params.ay_min) || !std::isfinite(params.ay_max) ||
            !std::isfinite(params.az_max) || params.ax_min > 0.0 ||
            params.ax_max < 0.0 || params.ay_min > 0.0 || params.ay_max < 0.0 ||
            params.az_max < 0.0) {
            throw std::invalid_argument(
                "Acceleration limits must satisfy ax_min <= 0 <= ax_max, "
                "ay_min <= 0 <= ay_max, and az_max >= 0");
        }

        registerCritic(std::make_unique<critics::ConstraintCritic>());
        critics::CostCriticExecutionOptions cost_critic_options;
        cost_critic_options.num_threads = static_cast<std::size_t>(
          params.CostCritic.num_threads);
        cost_critic_options.chunk_size = static_cast<std::size_t>(
          params.CostCritic.chunk_size);
        registerCritic(std::make_unique<critics::CostCritic>(cost_critic_options));
        registerCritic(std::make_unique<critics::GoalAngleCritic>());
        registerCritic(std::make_unique<critics::GoalCritic>());
        registerCritic(std::make_unique<critics::PathAlignCritic>());
        registerCritic(std::make_unique<critics::PathAngleCritic>());
        registerCritic(std::make_unique<critics::PathFollowCritic>());
        registerCritic(std::make_unique<critics::PreferForwardCritic>());
        registerCritic(std::make_unique<critics::TwirlingCritic>());
        registerCritic(std::make_unique<critics::VelocityDeadbandCritic>());

        visualizer_enabled_ = params.visualize;
        if (params.visualize) {
            if (!rclcpp::ok()) {
                throw std::runtime_error(
                    "MPPI visualization requires an initialized ROS context");
            }
            if (!visualizer_node_) {
                rclcpp::NodeOptions node_options;
                node_options.use_global_arguments(false);
                node_options.enable_rosout(false);
                node_options.start_parameter_services(false);
                node_options.start_parameter_event_publisher(false);
                visualizer_node_ = rclcpp::Node::make_shared(
                    params.visualizer_node_name, node_options);
            }
            VisualizerConfig visualizer_config;
            visualizer_config.frame_id = params.visualizer_frame_id;
            visualizer_config.trajectory_topic = params.visualizer_trajectory_topic;
            visualizer_config.optimal_path_topic = params.visualizer_optimal_path_topic;
            visualizer_config.reference_path_topic = params.visualizer_reference_path_topic;
            visualizer_config.trajectory_step = static_cast<std::size_t>(
              params.visualizer_trajectory_step);
            visualizer_config.time_step = static_cast<std::size_t>(params.visualizer_time_step);
            visualizer_config.max_trajectories = static_cast<std::size_t>(
              params.visualizer_max_trajectories);
            visualizer_config.publish_rate = params.visualizer_publish_rate;
            visualizer_.configure(visualizer_node_, visualizer_config);
        } else {
            visualization_generation_.fetch_add(1, std::memory_order_relaxed);
            visualizer_.reset();
        }

        std::cout<<"[nav2_mppi_controller] Critics registered. Initializing optimizer..."<<std::endl;
        std::cout<<"[nav2_mppi_controller] Number of critics: "<<critics_.size()<<std::endl;

        optimizer_.initialize(
            "MPPIController",
            critics_,
            params
            );
       //TODO:初始化过程需要完善
    }


     std::optional<finenav::DataPacket<OutputProfile>>plan(const finenav::PlanningContext<MPPIController>& ctx, const IMapView& map_view) {
     std::optional<finenav::core::Trajectory<OutputProfile>>plan(const finenav::core::PlanningContext<MPPIController>& ctx, const IMapView& map_view) {
        using Clock = std::chrono::steady_clock;
        using Ms    = std::chrono::duration<double, std::milli>;
        const auto t_plan_start = Clock::now();

        finenav::core::Trajectory<OutputProfile> traj;
        if (ctx.ref_traj.poses.empty()) {
            return std::nullopt;
        }

        finenav::DataPacket<OutputProfile> traj;
        // 从框架注入的 RobotState 中取出 pose 和 twist，无需 TF 查询
        const auto& robot_state = ctx.robot_state;
        auto goal_pose = ctx.goal_pose;

        nav_msgs::msg::Path path = convertDataPacketToPath(ctx.ref_traj, "map");
        this->setPlan(path);

        geometry_msgs::msg::PoseStamped start_pose_stamped;
        start_pose_stamped.pose.position.x = robot_state.position.x();
        start_pose_stamped.pose.position.y = robot_state.position.y();
        start_pose_stamped.pose.position.z = robot_state.position.z();
        start_pose_stamped.pose.orientation.w = robot_state.orientation.w();
        start_pose_stamped.pose.orientation.x = robot_state.orientation.x();
        start_pose_stamped.pose.orientation.y = robot_state.orientation.y();
        start_pose_stamped.pose.orientation.z = robot_state.orientation.z();
        start_pose_stamped.header.stamp = rclcpp::Clock().now();
        start_pose_stamped.header.frame_id = "map";

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = robot_state.linear_velocity.x();
        twist.linear.y  = robot_state.linear_velocity.y();
        twist.linear.z  = robot_state.linear_velocity.z();
        twist.angular.x = robot_state.angular_velocity.x();
        twist.angular.y = robot_state.angular_velocity.y();
        twist.angular.z = robot_state.angular_velocity.z();

        const auto t_optimize_start= Clock::now();
        auto optimization_result = computeVelocityCommands(
            start_pose_stamped,
            twist,
            map_view
            );
        const double ms_optimize = Ms(Clock::now() - t_optimize_start).count();

        if (visualizer_enabled_) {
            visualizer_.submit(
                optimizer_.getGeneratedTrajectories(),
                optimizer_.getCriticCostBreakdown(),
                optimization_result.trajectory,
                path,
                visualization_generation_.load(std::memory_order_relaxed));
        }

        // 装填输出轨迹

    // 获取最优轨迹
    const auto & optimized_trajectory = optimization_result.trajectory;
    const auto & control_seq = optimization_result.controls;
    // 填充轨迹点 (pose + twist)
    const size_t n = optimized_trajectory.rows();
    traj.poses.reserve(n);
    traj.twists.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        // pose: x, y, yaw -> quaternion
        geometry_msgs::msg::Pose pose;
        pose.position.x = optimized_trajectory(i, 0);
        pose.position.y = optimized_trajectory(i, 1);
        pose.position.z = 0.0;
        const float yaw = optimized_trajectory(i, 2);
        pose.orientation.z = std::sin(yaw * 0.5f);
        pose.orientation.w = std::cos(yaw * 0.5f);
        traj.poses.push_back(pose);

        // twist: vx, vy, wz from control sequence
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = control_seq(i, 0);
        twist.angular.z = control_seq(i, 1);
        twist.linear.y  = control_seq(i, 2);
        traj.twists.push_back(twist);
    }

    const double ms_total = Ms(Clock::now() - t_plan_start).count();
    {
        static rclcpp::Clock throttle_clk{RCL_STEADY_TIME};
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("MPPIController"), throttle_clk, 2000,
            "\n[MPPIController] plan() timing\n"
            "  evalControl (optimize) : %7.2f ms\n"
            "  total plan()           : %7.2f ms",
            ms_optimize, ms_total);
    }

    return traj;
}


  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path);

  /**
   * @brief Reset optimizer state at Episode boundary (new goal).
   *        Called by ControlLayer whenever a new Action Goal arrives.
   *        Clears control_sequence_ so the next plan() call starts from
   *        a zero warm-start, independent of any previous planning direction.
   */
  void reset() {
      // Called by ControlLayer at the episode boundary on the same worker as plan().
      const auto generation = visualization_generation_.fetch_add(
        1, std::memory_order_relaxed) + 1;
      if (visualizer_enabled_) {
        visualizer_.clear(generation);
      }
      optimizer_.reset();
  }

    void registerCritic (std::unique_ptr<critics::CriticFunction> critic) {
        critics_.push_back(std::move(critic));        //TODO:在初始化的时候要换成注册
    }


    template <typename DataPacketT>
    nav_msgs::msg::Path convertDataPacketToPath(
        const DataPacketT & data,
        const std::string & frame_id)
    {
        nav_msgs::msg::Path path;

        path.header.frame_id = frame_id;
        path.header.stamp = rclcpp::Clock().now();

        // ⭐ 关键：只有当有 poses 才能用
        if constexpr (requires { data.poses; }) {

            path.poses.reserve(data.poses.size());

            for (const auto & pose : data.poses) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path.header;
                ps.pose = pose;

                path.poses.push_back(ps);
            }
        }

        return path;
    }


protected:
    /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
	// geometry_msgs::msg::TwistStamped computeVelocityCommands(
    OptimizationResult computeVelocityCommands(
    	const geometry_msgs::msg::PoseStamped & robot_pose,
  		const geometry_msgs::msg::Twist & robot_speed,
    	const IMapView& map_view)
	{
	//  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
	//    reset();
	//  }
//  	last_time_called_ = clock_->now();

	// geometry_msgs::msg::TwistStamped cmd =
    auto result = optimizer_.evalControl(robot_pose, robot_speed, path_ ,map_view);

    return result;
}

    nav_msgs::msg::Path path_;

    std::shared_ptr<rclcpp::Node> visualizer_node_;
    TrajectoryVisualizer visualizer_;
    bool visualizer_enabled_{false};
    std::atomic<std::uint64_t> visualization_generation_{0};
    Optimizer optimizer_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

};

}  // namespace nav2_mppi_controller


FINENAV_REGISTER_ALGO_CONFIG(nav2_mppi_controller::MPPIController, finenav_mppi_controller)


#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
