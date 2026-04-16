// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_engine/finenav_engine.hpp"
#include "grid_map.hpp" // TODO: resolve anti-pattern export

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace finenav;
using MyPlanners = PlannerSet<>; // TODO: check whether set is empty

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("hello_finenav");

    auto finenav_engine = std::make_shared<FineNavEngine>(node);

    // ------------------------------ Launch Localization ----------------------------------
    LocalizerOptions localizer_options {
        .update_rate_hz        = 100,
        .max_delay_sec        = 0.5,
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
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        10,
        [&finenav_engine](const nav_msgs::msg::Odometry& msg) {
            // finenav_engine->getLocalizer().feedObservation(msg); // TODO: directly support odometry_msgs in the future
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header = msg.header;
            pose_msg.pose = msg.pose;

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
}
