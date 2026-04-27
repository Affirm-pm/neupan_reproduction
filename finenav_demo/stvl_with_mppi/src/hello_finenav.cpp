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
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include "finenav_mppi_controller/controller.hpp"
#include "stvl_manager.hpp"
#include "terrain_mapview.hpp"

using namespace finenav;
using std::chrono_literals::operator""ms;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.append_parameter_override("use_sim_time", true); // Use time in /clock
    auto node = rclcpp::Node::make_shared("hello_finenav", options); // TODO: inhirent options?

    auto finenav_engine = std::make_shared<FineNavEngine>(node);

    StvlManager stvl_manager(node);
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

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_buffer->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(
            node->get_node_base_interface(),
            node->get_node_timers_interface()));
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    map_server->registerPreUpdateHook([stvl_manager](GridMap<Voxel> & map) mutable {
        stvl_manager.PruneExpiredCells(map);
    });
    map_server->registerPostUpdateHook([node, map_cloud_pub, terrain_analyzer, finenav_engine](GridMap<Voxel> & map) {
        // terrain_analyzer->update() 在写锁内调用，costmap_image_ 的写受 MapServer 互斥锁隐式保护。
        // ControlLayer 在 plan() 时持有读锁，plan 路径上的 getCost() 读 costmap_image_ 也受同一锁保护。
        // 因此 costmap_image_ 无需额外加锁。
        terrain_analyzer->update(finenav_engine->getLocalizer().getState().p.z(), map);
        std::vector<Index> valid_indices;
        map.selectCellsByCondition(valid_indices, [](const Voxel & v) {
            return std::isfinite(v.z_height);
        });
        if (valid_indices.empty()) {
            return;
        }
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = node->now();
        cloud.header.frame_id = "map";
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(valid_indices.size());
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(valid_indices.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

        for (const auto & idx : valid_indices) {
            const auto pos = map.getPosition(idx);
            const auto & cell = map.at(idx);
            *iter_x = static_cast<float>(pos.x());
            *iter_y = static_cast<float>(pos.y());
            *iter_z = cell.z_height;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        map_cloud_pub->publish(cloud);
    });

    MapServer<GridMap<Voxel>>::SourceBufferPolicy policy;
    policy.max_buffer_size = 50000;
    policy.observation_keep_time_sec = 2.0;

    auto laser_injector = map_server->addUpdateSource<Pointcloud>(
        "front_laser",
        policy,
        [stvl_manager](const Pointcloud & msg, GridMap<Voxel> & map) mutable {
            stvl_manager.InsertPointCloud(msg, map);
        });

    auto cloud_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        node, "/cloud_for_lio");

    auto tf_filter = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        *cloud_sub,
        *tf_buffer,
        "map",
        10,
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        10ms);

    tf_filter->registerCallback(
        [node, laser_injector, tf_buffer](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
            geometry_msgs::msg::TransformStamped tf_stamped;
            try {
                tf_stamped = tf_buffer->lookupTransform("map", msg->header.frame_id, msg->header.stamp);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(node->get_logger(), "TF lookup failed in cloud callback: %s", ex.what());
                return;
            }

            const Eigen::Isometry3d T_map_sensor = tf2::transformToEigen(tf_stamped.transform);
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
            const rclcpp::Time now = node->now();

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                    continue;
                }

                const Eigen::Vector3d p_sensor(*iter_x, *iter_y, *iter_z);
                const Eigen::Vector3d p_map = T_map_sensor * p_sensor;
                Pointcloud p;
                p.x = static_cast<float>(p_map.x());
                p.y = static_cast<float>(p_map.y());
                p.z = static_cast<float>(p_map.z());
                laser_injector(std::move(p), now);
            }
        });



    // --------------------------------- Launch Planning --------------------------------------
    using MppiPlanners = PlannerSet<nav2_mppi_controller::MPPIController>;
    auto my_control_layer= finenav_engine->createControlLayer<MppiPlanners>("mppi_layer");

    // my_control_layer->bind_map_view(
    //     mppi_map_server,
    //     [node](const auto& map) {
    //         return std::make_shared<SimpleGridMapView>(
    //             const_cast<GridMap<Voxel>&>(map), node.get());
    //     });

    // MapView构造用非const的map引用是一个反模式，这不是框架的问题，而是出自用户的
    // 就好比C++提供了各种强制类型转换的工具，而用户不按类型写代码，到处使用强制类型转换
    // 如果MapView构造使用了const的map引用，bind_map_view的调用会极其简洁


    my_control_layer->bind_map_view(
       map_server,
        [node, terrain_analyzer](const GridMap<Voxel>& map) {
            return std::make_shared<SimpleGridMapView>(map, node, terrain_analyzer); // 这里顺手把构造函数中的node从node指针变为node共享指针
        });


    finenav_engine->registerPipeline(
        FineNavEngine::PipelineIO{
            .input_topic = "line_path",
            .output_topic = "mppi_layer_plan",
        },
        my_control_layer);

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
