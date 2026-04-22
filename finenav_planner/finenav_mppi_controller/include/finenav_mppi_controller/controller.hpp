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

//#include "finenav_mppi_controller/tools/path_handler.hpp"
//#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"
#include "finenav_mppi_controller/models/constraints.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

#include "rclcpp/rclcpp.hpp"


#include "finenav_interface/test_map_params.hpp"
#include "finenav_interface/test_planner_params.hpp"

#include "finenav_core/concepts.hpp"
#include "finenav_util/algo_configurator.hpp"
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

    using IMapView = nav2_mppi_controller::IMapView;
    using Context = finenav::PlanningContext<MPPIController>;
    using ConfigType = finenav_mppi_controller::Params;
    void configure(const ConfigType& params) {              //应该传入参数 因测试先去除

        registerCritic(std::make_unique<critics::ConstraintCritic>());
        registerCritic(std::make_unique<critics::CostCritic>());
        registerCritic(std::make_unique<critics::GoalAngleCritic>());
        registerCritic(std::make_unique<critics::GoalCritic>());
        registerCritic(std::make_unique<critics::ObstaclesCritic>());
        registerCritic(std::make_unique<critics::PathAlignCritic>());
        registerCritic(std::make_unique<critics::PathAlignLegacyCritic>());
        registerCritic(std::make_unique<critics::PathAngleCritic>());
        registerCritic(std::make_unique<critics::PathFollowCritic>());
        registerCritic(std::make_unique<critics::PreferForwardCritic>());
        registerCritic(std::make_unique<critics::TwirlingCritic>());
        registerCritic(std::make_unique<critics::VelocityDeadbandCritic>());

        std::cout<<"[nav2_mppi_controller] Critics registered. Initializing optimizer..."<<std::endl;
        std::cout<<"[nav2_mppi_controller] Number of critics: "<<critics_.size()<<std::endl;


        optimizer_.initialize(
            "MPPIController",
            critics_
            );
       //TODO:初始化过程需要完善
    }

     std::optional<finenav::DataPacket<OutputProfile>>plan(const finenav::PlanningContext<MPPIController>& ctx, const IMapView& map_view) {

        finenav::DataPacket<OutputProfile> traj;
        auto start_pose = ctx.start_pose;
        auto goal_pose = ctx.goal_pose;

        nav_msgs::msg::Path path = convertDataPacketToPath(ctx.ref_traj, "map");

        this->setPlan(path);

        std::cout << "[finenav_mppi_controller] Planning from ("
                  << start_pose.position.x << ", " << start_pose.position.y << ") to ("
                  << goal_pose.position.x << ", " << goal_pose.position.y << ")" << std::endl;
        geometry_msgs::msg::PoseStamped start_pose_stamped;
        start_pose_stamped.pose = start_pose;
        start_pose_stamped.header.stamp = rclcpp::Clock().now();
        start_pose_stamped.header.frame_id = "map"; // 根据实际需求设置 frame_id

        computeVelocityCommands(
            start_pose_stamped,
            geometry_msgs::msg::Twist(),  //TODO: 传入实际速度，先用零速度占位
            map_view
            );
        // 装填输出轨迹

    // 获取最优轨迹
    auto optimized_trajectory = optimizer_.getOptimizedTrajectory();
    // 填充轨迹点
    for (size_t i = 0; i < optimized_trajectory.shape(0); ++i) {
        traj.poses.emplace_back();
        traj.poses.back().position.x = optimized_trajectory(i, 0);
        traj.poses.back().position.y = optimized_trajectory(i, 1);
    }

    return traj;
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
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
//    void visualize(
//        nav_msgs::msg::Path transformed_plan,
//        const builtin_interfaces::msg::Time & cmd_stamp,
//        double z_height);

    /**
    * @brief Main method to compute velocities using the optimizer
    * @param robot_pose Robot pose
    * @param robot_speed Robot speed
    * @param goal_checker Pointer to the goal checker for awareness if completed task
    */
	// geometry_msgs::msg::TwistStamped computeVelocityCommands(
    void computeVelocityCommands(
    	const geometry_msgs::msg::PoseStamped & robot_pose,
  		const geometry_msgs::msg::Twist & robot_speed,
    	const IMapView& map_view)
	{
	#ifdef BENCHMARK_TESTING
  	auto start = std::chrono::system_clock::now();
	#endif

	//  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
	//    reset();
	//  }
//  	last_time_called_ = clock_->now();

//  	std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
//

    std::cout<<"[nav2_mppi_controller] Computing velocity commands..."<<std::endl;
  	// geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, path_ ,map_view);

	#ifdef BENCHMARK_TESTING
  	auto end = std::chrono::system_clock::now();
  	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  	RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
	#endif

//  	if (visualize_) {
//    	double z_height = 0.06;
//    	const std::string base_frame = map_view->getBaseFrameID();
//    	try {
//      	auto tfst = tf_buffer_->lookupTransform(
//        	map_view.getGlobalFrameID(), base_frame,  cmd.header.stamp);
//      		z_height = tfst.transform.translation.z;
//    	} catch (const tf2::TransformException & ex) {
//      	RCLCPP_WARN(logger_, "Failed to get transform %s -> %s: %s",
//                  map_view.getGlobalFrameID().c_str(), base_frame.c_str(), ex.what());
//    	}
//    	visualize(std::move(path_, cmd.header.stamp, z_height);
//  	}
  	// return cmd;
}

//  std::string name_;
//  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
//  rclcpp::Clock::SharedPtr clock_;
//  rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
//  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path path_;

//  std::unique_ptr<ParametersHandler> parameters_handler_;
  Optimizer optimizer_;
//  TrajectoryVisualizer trajectory_visualizer_;
  std::vector<std::unique_ptr<critics::CriticFunction>> critics_;

  bool visualize_;

  double reset_period_;
  // Last time computeVelocityCommands was called
//  rclcpp::Time last_time_called_;
};

}  // namespace nav2_mppi_controller


FINENAV_REGISTER_ALGO_CONFIG(nav2_mppi_controller::MPPIController, finenav_mppi_controller)


#endif  // NAV2_MPPI_CONTROLLER__CONTROLLER_HPP_
