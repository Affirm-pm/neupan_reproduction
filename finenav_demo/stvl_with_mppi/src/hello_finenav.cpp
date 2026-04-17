// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <nav_msgs/msg/detail/odometry__struct.hpp>

#include "finenav_engine/finenav_engine.hpp"
#include "grid_map.hpp" // TODO: resolve anti-pattern export

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace finenav;
using MyPlanners = PlannerSet<>; // TODO: check whether set is empty

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true); // Use time in /clock
    auto node = rclcpp::Node::make_shared("hello_finenav", options); // TODO: inhirent options?

    auto finenav_engine = std::make_shared<FineNavEngine>(node);

    // ------------------------------ Launch Localization ----------------------------------
    LocalizerOptions localizer_options {
        .update_rate_hz        = 100,
        .max_delay_sec        = 0.05,
        .pose_smoothing_steps = 1,
        .twist_smoothing_steps = 1,
        .pose_gate_dist = 1e9,
        .twist_gate_dist = 1e9,
        .model_opts = {
            .vel_noise_std = 0.1,
            .omega_noise_std = 0.1
        }
    };
    finenav_engine->getLocalizer(localizer_options); // 【注意：目前存在小bug，必须将这个api在createMapResource前调用一次】
    auto gt_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "ground_truth_pose",
        10,
        [&finenav_engine, &node](const geometry_msgs::msg::PoseStamped& msg) {
            // finenav_engine->getLocalizer().feedObservation(msg); // TODO: directly support odometry_msgs in the future
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg.header;
            pose_msg.pose.pose = msg.pose;

            // auto time = node->now() - msg.header.stamp;
            // RCLCPP_INFO_STREAM(node->get_logger(), "Received GT pose with delay " << time.seconds() << " seconds.");

            static bool localizer_initialized = false;      // TODO: automatically iniitailize when receive the first pose_msg
            if (!localizer_initialized) {
                finenav_engine->getLocalizer().setInitialPose(pose_msg);
                localizer_initialized = true;
            } else {
                finenav_engine->getLocalizer().feedObservation(pose_msg);
            }
        }
    );


    // --------------------------------- Launch Mapping --------------------------------------
    auto map_server = finenav_engine->createMapResource<GridMap<float>>(
        "my_map_server",
        10.0,
        true,
        100.0
    );

    // .......

    // --------------------------------- Launch Planning --------------------------------------

    // auto my_control_layer = finenav_engine->createControlLayer<MyPlanners>("my_control_layer");
    // finenav_engine->registerPipeline(FineNavEngine::PipelineIO{}, my_control_layer);

    // .......

    // --------------------------------- Main Loop --------------------------------------------
    // ---- 1.Publish current state using nav_msgs/msg/Odometry
    // ---- 2.

    rclcpp::Rate loop_rate(200); // 200Hz
    auto nav_state_pub = node->create_publisher<nav_msgs::msg::Odometry>("nav_state", 10);

    while (rclcpp::ok()) {
        auto robot_state = finenav_engine->getLocalizer().getState();

        // Initialize with IIFE
        const auto nav_state_msg = [&node, &robot_state] {
            nav_msgs::msg::Odometry nav_state_msg;
            nav_state_msg.header.stamp = node->now();
            nav_state_msg.header.frame_id = "map";
            nav_state_msg.child_frame_id = "base_link";

            nav_state_msg.pose.pose.position.x = robot_state.p.x();
            nav_state_msg.pose.pose.position.y = robot_state.p.y();
            nav_state_msg.pose.pose.position.z = robot_state.p.z();

            auto quat = robot_state.R.unit_quaternion();
            nav_state_msg.pose.pose.orientation.x = quat.x();
            nav_state_msg.pose.pose.orientation.y = quat.y();
            nav_state_msg.pose.pose.orientation.z = quat.z();
            nav_state_msg.pose.pose.orientation.w = quat.w();

            nav_state_msg.twist.twist.linear.x = robot_state.v.x();
            nav_state_msg.twist.twist.linear.y = robot_state.v.y();
            nav_state_msg.twist.twist.linear.z = robot_state.v.z();
            nav_state_msg.twist.twist.angular.x = robot_state.omega.x();
            nav_state_msg.twist.twist.angular.y = robot_state.omega.y();
            nav_state_msg.twist.twist.angular.z = robot_state.omega.z();

            return nav_state_msg;
        }();
        nav_state_pub->publish(nav_state_msg);


        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
