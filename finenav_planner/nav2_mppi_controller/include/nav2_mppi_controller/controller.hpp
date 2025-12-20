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

#include <string>
#include <memory>

#include "nav2_mppi_controller/tools/path_handler.hpp"
#include "nav2_mppi_controller/optimizer.hpp"
#include "nav2_mppi_controller/tools/trajectory_visualizer.hpp"
#include "nav2_mppi_controller/models/constraints.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"

#include "finenav_util/algo_configurator.hpp"
#include "finenav_util/planner_traits.hpp"
#include "finenav_core/concepts.hpp"
#include "nav2_mppi_controller/finenav_mppi_controller_params.hpp"

namespace nav2_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */
class MPPIController : public nav2_core::Controller
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
  MPPIController() = default;

  /**
    * @brief Configure controller on bringup
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param tf TF buffer to use
    * @param costmap_ros Costmap2DROS object of environment
    */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
    * @brief Cleanup resources
    */
  void cleanup() override;

  /**
    * @brief Activate controller
    */
  void activate() override;

  /**
    * @brief Deactivate controller
    */
  void deactivate() override;

  /**
    * @brief Reset the controller state between tasks
    */
  void reset();

  /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    nav2_core::GoalChecker * goal_checker) override;

  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;


    using ConfigType = finenav_mppi_controller::Params;

    void configure(const ConfigType &params) {
        if (params.background.b == 255) { /* 复杂的分支控制逻辑 */}
    }

    template <typename MapView>
    bool plan(const MapView& map, const finenav::PlanningContext& ctx, finenav_msgs::msg::Trajectory& output_traj) {

        auto start_pose = ctx.start_pose;
        auto goal_pose = ctx.goal_pose;
        auto trajectory = finenav_msgs::msg::Trajectory(); // 这里会有一个 helper function 帮忙解包

        // 执行规划算法，e.g. 生成一条直线路径
        std::cout << "[nav2_mppi_controller] Planning from ("
                  << start_pose.position.x << ", " << start_pose.position.y << ") to ("
                  << goal_pose.position.x << ", " << goal_pose.position.y << ")" << std::endl;

        // 装填输出轨迹
        return true;
    }

protected:
  /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
  void visualize(
    nav_msgs::msg::Path transformed_plan,
    const builtin_interfaces::msg::Time & cmd_stamp,
    double z_height);

  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;

  bool visualize_;

  double reset_period_;
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;
};

}  // namespace nav2_mppi_controller


FINENAV_REGISTER_ALGO_CONFIG(nav2_mppi_controller::MPPIController, finenav_mppi_controller)
FINENAV_REGISTER_PLANNER_TRAITS(nav2_mppi_controller::MPPIController,
    finenav::TraitMask::NONE,
    finenav::TraitMask::POSE)

#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
