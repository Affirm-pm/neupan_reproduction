// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "finenav_engine/finenav_engine.hpp"
#include "grid_map.hpp" // TODO: resolve anti-pattern export

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "finenav_mppi_controller/controller.hpp"
#include "stvl_manager.hpp"
#include "terrain_mapview.hpp"

using namespace finenav;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true); // Use time in /clock
    auto node = rclcpp::Node::make_shared("hello_finenav", options); // TODO: inhirent options?

    auto finenav_engine = std::make_shared<FineNavEngine>(node);

    // 雷达相对车体的安装位姿（外参）。默认 Identity 表示雷达与车体原点重合。
    // TODO: 从 ROS 参数或 URDF 读取
    Eigen::Isometry3d T_body_lidar = Eigen::Isometry3d::Identity();

    auto stvl_manager = std::make_shared<StvlManager>(node, T_body_lidar);
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
    auto map_server = finenav_engine->createMapResource<GridMap<Voxel>>(
        "my_map_server", 10.0, true, 100.0);


    auto terrain_analyzer = std::make_shared<TerrainAnalyzer>(map_server, node);

    auto map_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 10);


    map_server->registerPreUpdateHook([stvl_manager, &finenav_engine](GridMap<Voxel> & map) {
        const auto & state = finenav_engine->getLocalizer().getState();
        Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
        T_map_body.linear() = state.R.matrix();
        T_map_body.translation() = state.p;
        stvl_manager->PruneExpiredCells(map, T_map_body);
    });
    map_server->registerPostUpdateHook([node, map_cloud_pub, terrain_analyzer, finenav_engine](GridMap<Voxel> & map) {

        terrain_analyzer->update(finenav_engine->getLocalizer().getState().p.z(), map);

        // 可视化
        std::vector<Index> valid_indices;
        map.selectCellsByCondition(valid_indices, [](const Voxel & v) {
            return std::isfinite(v.z_height);
        });
        if (valid_indices.empty()) {
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud;
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.reserve(valid_indices.size());
        for (const auto & idx : valid_indices) {
            const auto pos = map.getPosition(idx);
            const auto & cell = map.at(idx);
            pcl_cloud.emplace_back(
                static_cast<float>(pos.x()),
                static_cast<float>(pos.y()),
                cell.z_height);
        }
        pcl::toROSMsg(pcl_cloud, cloud);
        cloud.header.stamp = node->now();
        cloud.header.frame_id = "map";

        map_cloud_pub->publish(cloud);
    });

    MapServer<GridMap<Voxel>>::SourceBufferPolicy policy;
    policy.max_buffer_size = 50000;
    policy.observation_keep_time_sec = 2.0;

    auto laser_injector = map_server->addUpdateSource<pcl::PointCloud<pcl::PointXYZ>>(
        "front_laser",
        policy,
        [stvl_manager](const pcl::PointCloud<pcl::PointXYZ> & cloud, GridMap<Voxel> & map) {
            stvl_manager->InsertPointCloud(cloud, map);
        });

    // TODO: 输入点云默认在车体坐标系下，无需 TF 变换；即使对于raycast一类需要雷达物理安装位置的应用，也可以通过读tf的方式来获取
    auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_for_lio",
        10,
        [node, laser_injector](const sensor_msgs::msg::PointCloud2 & msg) {
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::fromROSMsg(msg, pcl_cloud);
            laser_injector(std::move(pcl_cloud), rclcpp::Time(msg.header.stamp)); // TODO： 输入时间戳应该是 builtin_msgs，而不是 rclcpp::Time
        });



    // --------------------------------- Launch Planning --------------------------------------
    using MppiPlanners = PlannerSet<nav2_mppi_controller::MPPIController>;
    auto my_control_layer= finenav_engine->createControlLayer<MppiPlanners>("mppi_layer");

    // TODO: 函数签名对静态工具支持不友好，后续需要改
    my_control_layer->bind_map_view(
       map_server,
        [node, terrain_analyzer](const GridMap<Voxel>& map) {
            return std::make_shared<SimpleGridMapView>(map, node, terrain_analyzer);
        });


    finenav_engine->registerPipeline(
        FineNavEngine::PipelineIO{
            .input_topic = "line_path",
            .output_topic = "mppi_layer_plan",
        },
        my_control_layer);


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
