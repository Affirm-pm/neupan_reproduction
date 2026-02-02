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

#include "finenav_mppi_controller/tools/path_handler.hpp"
#include "finenav_mppi_controller/optimizer.hpp"
#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"
#include "finenav_mppi_controller/models/constraints.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"


#include "finenav_interface/test_map_params.hpp"
#include "finenav_interface/test_planner_params.hpp"

#include "finenav_core/concepts.hpp"
#include "finenav_util/trajectory_io/data_packet.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_mppi_controller/finenav_mppi_controller_params.hpp"

namespace nav2_mppi_controller
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIController
 * @brief Main plugin controller for MPPI Controller
 */
template <typename MapView>
class MPPIController
{
public:
  /**
    * @brief Constructor for mppi::MPPIController
    */
    MPPIController() = default;

    static constexpr finenav::IOProfile InputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };
    static constexpr finenav::IOProfile OutputProfile = {
        .pose = true,
        .vel  = false,
        .accel= false,
        .time = false
    };

    using Context = finenav::PlanningContext<MPPIController>;
    using Result  = std::optional<finenav::OutputDataT<MPPIController>>;
    using ConfigType = finenav_mppi_controller::Params;
    void configure(const ConfigType &params) {
        if (params.background.b == 255) { /* 复杂的分支控制逻辑 */}    //TODO:初始化过程需要完善
    }

    bool plan(const MapView& map_view, const finenav::PlanningContext<MPPIController>& ctx, finenav_msgs::msg::Trajectory& output_traj) {

        auto start_pose = ctx.start_pose;
        auto goal_pose = ctx.goal_pose;
        auto trajectory = finenav_msgs::msg::Trajectory(); // 这里会有一个 helper function 帮忙解包

        // 执行规划算法，e.g. 生成一条直线路径
        std::cout << "[nav2_mppi_controller] Planning from ("
                  << start_pose.position.x << ", " << start_pose.position.y << ") to ("
                  << goal_pose.position.x << ", " << goal_pose.position.y << ")" << std::endl;
        geometry_msgs::msg::PoseStamped start_pose_stamped;
        start_pose_stamped.pose = start_pose;
        start_pose_stamped.header.stamp = rclcpp::Clock().now();
        start_pose_stamped.header.frame_id = "map"; // 根据实际需求设置 frame_id


        computeVelocityCommands(
            start_pose_stamped,
            geometry_msgs::msg::Twist(),
            map_view
            );
        // 装填输出轨迹

    // 获取最优轨迹
    auto optimized_trajectory = optimizer_.getOptimizedTrajectory();

    // 清空旧数据
    output_traj.poses.clear();
    output_traj.twists.clear();
    output_traj.accels.clear();
    output_traj.time_from_start.clear();

    // 填充 header
    output_traj.header.stamp = rclcpp::Clock().now();
    output_traj.header.frame_id = "map";

    // 填充轨迹点
    for (size_t i = 0; i < optimized_trajectory.shape(0); ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = optimized_trajectory(i, 0);
        pose.position.y = optimized_trajectory(i, 1);
        if (optimized_trajectory.shape(1) > 2) {
            pose.position.z = optimized_trajectory(i, 2);
        }
        output_traj.poses.push_back(pose);

        // 如果有速度、加速度或时间数据，填充对应字段
        if (optimized_trajectory.shape(1) > 3) {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = optimized_trajectory(i, 3);
            twist.linear.y = optimized_trajectory(i, 4);
            output_traj.twists.push_back(twist);
        }
        if (optimized_trajectory.shape(1) > 5) {
            geometry_msgs::msg::Accel accel;
            accel.linear.x = optimized_trajectory(i, 5);
            accel.linear.y = optimized_trajectory(i, 6);
            output_traj.accels.push_back(accel);
        }
        if (optimized_trajectory.shape(1) > 7) {
            builtin_interfaces::msg::Duration time;
            time.sec = static_cast<int32_t>(optimized_trajectory(i, 7));
            time.nanosec = static_cast<uint32_t>((optimized_trajectory(i, 7) - time.sec) * 1e9);
            output_traj.time_from_start.push_back(time);
        }
    }

        return true;
    }


  /**
    * @brief Set new reference path to track
    * @param path Path to track
    */
  void setPlan(const nav_msgs::msg::Path & path);

  /**
    * @brief Set new speed limit from callback
    * @param speed_limit Speed limit to use
    * @param percentage Bool if the speed limit is absolute or relative
    */
  void setSpeedLimit(const double & speed_limit, const bool & percentage);

  void registerCritic(std::unique_ptr<critics::CriticFunction<MapView>> critic) {
        critics_.push_back(std::move(critic));        //TODO:在初始化的时候要换成注册
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

    /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
	geometry_msgs::msg::TwistStamped computeVelocityCommands(
    	const geometry_msgs::msg::PoseStamped & robot_pose,
  		const geometry_msgs::msg::Twist & robot_speed,
    	const MapView& map_view)
	{
	#ifdef BENCHMARK_TESTING
  	auto start = std::chrono::system_clock::now();
	#endif

	//  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
	//    reset();
	//  }
  	last_time_called_ = clock_->now();

  	std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  	nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose, map_view);


  	geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan ,map_view);

	#ifdef BENCHMARK_TESTING
  	auto end = std::chrono::system_clock::now();
  	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  	RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
	#endif

  	if (visualize_) {
    	double z_height = 0.06;
    	const std::string base_frame = map_view->getBaseFrameID();
    	try {
      	auto tfst = tf_buffer_->lookupTransform(
        	map_view.getGlobalFrameID(), base_frame,  cmd.header.stamp);
      		z_height = tfst.transform.translation.z;
    	} catch (const tf2::TransformException & ex) {
      	RCLCPP_WARN(logger_, "Failed to get transform %s -> %s: %s",
                  map_view.getGlobalFrameID().c_str(), base_frame.c_str(), ex.what());
    	}
    	visualize(std::move(transformed_plan), cmd.header.stamp, z_height);
  	}
  	return cmd;
}

  std::string name_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer<MapView> optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;
  std::vector<std::unique_ptr<critics::CriticFunction<MapView>>> critics_;

  bool visualize_;

  double reset_period_;
  // Last time computeVelocityCommands was called
  rclcpp::Time last_time_called_;
};

}  // namespace nav2_mppi_controller


FINENAV_REGISTER_TEMPLATE_ALGO_CONFIG(nav2_mppi_controller::MPPIController, finenav_mppi_controller)


#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
