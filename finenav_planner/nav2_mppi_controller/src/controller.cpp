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

#include <stdint.h>
#include <chrono>
#include <tf2/exceptions.h>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{
template <typename MapView>
geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
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
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

	nav2_costmap_2d::Costmap2D * costmap ;
	unsigned int size_x = costmap->getSizeInCellsX();
	unsigned int size_y = costmap->getSizeInCellsY();

	for (unsigned int mx = 0; mx < size_x; ++mx) {
		for (unsigned int my = 0; my < size_y; ++my) {
			double wx, wy;
				costmap->mapToWorld(mx, my, wx, wy); // 将地图坐标转换为世界坐标

				Position3D pos{wx, wy, 0.0}; // 假设 z 坐标为 0
        		double cost = map_view.getCost(pos); // 获取代价

       	 		costmap->setCost(mx, my, static_cast<unsigned char>(cost)); // 设置代价
           }
		}
  		std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan ,map_view);

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  if (visualize_) {
    double z_height = 0.06;
    const std::string base_frame = costmap_ros_->getBaseFrameID();
    try {
      auto tfst = tf_buffer_->lookupTransform(
        map_view->getGlobalFrameID(), base_frame,  cmd.header.stamp);
      z_height = tfst.transform.translation.z;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Failed to get transform %s -> %s: %s",
                  map_view->getGlobalFrameID().c_str(), base_frame.c_str(), ex.what());
    }

    visualize(std::move(transformed_plan), cmd.header.stamp, z_height);
  }

  return cmd;
}

void MPPIController::visualize(
  nav_msgs::msg::Path transformed_plan,
  const builtin_interfaces::msg::Time & cmd_stamp,
  double z_height)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories", z_height);
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory", cmd_stamp, z_height);
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

//FINENAV_REGISTER_ALGO_CONFIG(nav2_mppi_controller::MPPIController, finenav_mppi_controller)
//FINENAV_REGISTER_PLANNER_TRAITS(nav2_mppi_controller::MPPIController,
//    finenav::TraitMask::NONE,
//    finenav::TraitMask::POSE)
}  // namespace nav2_mppi_controller

//#include "pluginlib/class_list_macros.hpp"
//PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
