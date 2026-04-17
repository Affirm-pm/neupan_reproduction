//// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//// Copyright (c) 2023 Dexory
//// Copyright (c) 2023 Open Navigation LLC
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.
//
//#ifndef NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
//#define NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
//
//#include <vector>
//#include <utility>
//#include <string>
//#include <memory>
//
//#include "rclcpp_lifecycle/lifecycle_node.hpp"
//#include "tf2_ros/buffer.h"
//#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "nav_msgs/msg/path.hpp"
//#include "builtin_interfaces/msg/time.hpp"
//
//#include "finenav_mppi_controller/tools/utils.hpp"
////#include "finenav_mppi_controller/tools/parameters_handler.hpp"
//
//namespace mppi
//{
//
//using PathIterator = std::vector<geometry_msgs::msg::PoseStamped>::iterator;
//using PathRange = std::pair<PathIterator, PathIterator>;
//
///**
// * @class mppi::PathHandler
// * @brief Manager of incoming reference paths for transformation and processing
// */
//
//class PathHandler
//{
//public:
//  /**
//    * @brief Constructor for mppi::PathHandler
//    */
//    PathHandler() = default;
//
//  /**
//    * @brief Destructor for mppi::PathHandler
//    */
//    ~PathHandler() = default;
//
//  /**
//    * @brief Initialize path handler on bringup
//    * @param parent WeakPtr to node
//    * @param name Name of plugin
//    * @param costmap_ros Costmap2DROS object of environment
//    * @param tf TF buffer for transformations
//    * @param dynamic_parameter_handler Parameter handler object
//    */
//    void initialize(
//    rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
//    std::shared_ptr<tf2_ros::Buffer>);
//
//  /**
//    * @brief Set new reference path
//    * @param Plan Path to use
//    */
//    void setPath(const nav_msgs::msg::Path & plan);
//
//  /**
//    * @brief Get reference path
//    * @return Path
//    */
//    nav_msgs::msg::Path & getPath();
//
//  /**
//   * @brief transform global plan to local applying constraints,
//   * then prune global plan
//   * @param robot_pose Pose of robot
//   * @return global plan in local frame
//   */
//    template <typename MapView>
//    nav_msgs::msg::Path transformPath(
//      const geometry_msgs::msg::PoseStamped & robot_pose,
//      const MapView& map_view)
//    {
//        // Find relevent bounds of path to use
//        geometry_msgs::msg::PoseStamped global_pose =
//          transformToGlobalPlanFrame(robot_pose);
//        auto [transformed_plan, lower_bound] = getGlobalPlanConsideringBoundsInCostmapFrame(global_pose,map_view);
//
//        prunePlan(global_plan_up_to_inversion_, lower_bound);
//
//        if (enforce_path_inversion_ && inversion_locale_ != 0u) {
//            if (isWithinInversionTolerances(global_pose)) {
//                prunePlan(global_plan_, global_plan_.poses.begin() + inversion_locale_);
//                global_plan_up_to_inversion_ = global_plan_;
//                inversion_locale_ = utils::removePosesAfterFirstInversion(global_plan_up_to_inversion_);
//            }
//        }
//
//        if (transformed_plan.poses.empty()) {
//            throw std::runtime_error("Resulting plan has 0 poses in it.");
//        }
//
//        return transformed_plan;
//    }
//protected:
//  /**
//    * @brief Transform a pose to another frame
//    * @param frame Frame to transform to
//    * @param in_pose Input pose
//    * @param out_pose Output pose
//    * @return Bool if successful
//    */
//    bool transformPose(
//        const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
//        geometry_msgs::msg::PoseStamped & out_pose) const;
//
//  /**
//    * @brief Get largest dimension of costmap (radially)
//    * @return Max distance from center of costmap to edge
//    */
//      double getMaxCostmapDist();
//
//  /**
//    * @brief Transform a pose to the global reference frame
//    * @param pose Current pose
//    * @return output poose in global reference frame
//    */
//    geometry_msgs::msg::PoseStamped
//        transformToGlobalPlanFrame(const geometry_msgs::msg::PoseStamped & pose);
//
//  /**
//    * @brief Get global plan within window of the local costmap size
//    * @param global_pose Robot pose
//    * @return plan transformed in the costmap frame and iterator to the first pose of the global
//    * plan (for pruning)
//    */
//    template <typename MapView>
//    std::pair<nav_msgs::msg::Path, PathIterator>
//    getGlobalPlanConsideringBoundsInCostmapFrame(
//        const geometry_msgs::msg::PoseStamped & global_pose,
//        const MapView& map_view)
//    {
//        using nav2_util::geometry_utils::euclidean_distance;
//
//        auto begin = global_plan_up_to_inversion_.poses.begin();
//
//        // Limit the search for the closest pose up to max_robot_pose_search_dist on the path
//        auto closest_pose_upper_bound =
//            nav2_util::geometry_utils::first_after_integrated_distance(
//            global_plan_up_to_inversion_.poses.begin(), global_plan_up_to_inversion_.poses.end(),
//            max_robot_pose_search_dist_);
//
//        // Find closest point to the robot
//        auto closest_point = nav2_util::geometry_utils::min_by(
//            begin, closest_pose_upper_bound,
//            [&global_pose](const geometry_msgs::msg::PoseStamped & ps) {
//            return euclidean_distance(global_pose, ps);
//        });
//
//    nav_msgs::msg::Path transformed_plan;
//    transformed_plan.header.frame_id = map_view.getGlobalFrameID();
//    transformed_plan.header.stamp = global_pose.header.stamp;
//
//    auto pruned_plan_end =
//        nav2_util::geometry_utils::first_after_integrated_distance(
//        closest_point, global_plan_up_to_inversion_.poses.end(), prune_distance_);
//
//    unsigned int mx, my;
//    // Find the furthest relevent pose on the path to consider within costmap
//    // bounds
//    // Transforming it to the costmap frame in the same loop
//    for (auto global_plan_pose = closest_point; global_plan_pose != pruned_plan_end;
//        ++global_plan_pose)
//    {
//    // Transform from global plan frame to costmap frame
//        geometry_msgs::msg::PoseStamped map_plan_pose;
//        global_plan_pose->header.stamp = global_pose.header.stamp;
//        global_plan_pose->header.frame_id = global_plan_.header.frame_id;
//        transformPose(map_view.getGlobalFrameID(), *global_plan_pose, map_plan_pose);
//
//        // Check if pose is inside the costmap
//        if (!map_view ->worldToMap(
//            map_plan_pose.pose.position.x, map_plan_pose.pose.position.y, mx, my))
//        {
//          return {transformed_plan, closest_point};
//        }
//
//        // Filling the transformed plan to return with the transformed pose
//        transformed_plan.poses.push_back(map_plan_pose);
//    }
//    return {transformed_plan, closest_point};
//}
//  /**
//    * @brief Prune a path to only interesting portions
//    * @param plan Plan to prune
//    * @param end Final path iterator
//    */
//    void prunePlan(nav_msgs::msg::Path & plan, const PathIterator end);
//
//  /**
//    * @brief Check if the robot pose is within the set inversion tolerances
//    * @param robot_pose Robot's current pose to check
//    * @return bool If the robot pose is within the set inversion tolerances
//    */
//    bool isWithinInversionTolerances(const geometry_msgs::msg::PoseStamped & robot_pose);
//
//    std::string name_;
//    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//
//    nav_msgs::msg::Path global_plan_;
//    nav_msgs::msg::Path global_plan_up_to_inversion_;
//    rclcpp::Logger logger_{rclcpp::get_logger("MPPIController")};
//
//    double max_robot_pose_search_dist_{0};
//    double prune_distance_{0};
//    double transform_tolerance_{0};
//    float inversion_xy_tolerance_{0.2};
//    float inversion_yaw_tolerance{0.4};
//    bool enforce_path_inversion_{false};
//    unsigned int inversion_locale_{0u};
//};
//}  // namespace mppi
//
//#endif  // NAV2_MPPI_CONTROLLER__TOOLS__PATH_HANDLER_HPP_
