// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-2-3.
//

#include "terrain_mapview.hpp"
#include <message_filters/subscriber.h>

#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.hpp>


#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <vector>


using namespace mppi;
using std::chrono_literals::operator"" ms;

bool numisfinite(float x) {
    if(x==std::numeric_limits<float>::infinity() || x==-std::numeric_limits<float>::infinity() || std::isnan(x)) {
        return false;
    }
    return true;
}

// 新增：将轨迹转换为点云的函数
sensor_msgs::msg::PointCloud2 trajectoryToPointCloud(const finenav_msgs::msg::Trajectory& traj) {
    sensor_msgs::msg::PointCloud2 pointcloud;
    if (traj.poses.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("traj"),
                    "Empty trajectory! Skip conversion.");
        return pointcloud;  // 返回空点云（安全）
    }

    // 设置点云头信息
    pointcloud.header = traj.header;
    pointcloud.height = 1;  // 无序点云，高度为1
    pointcloud.width = traj.poses.size();  // 宽度为点的数量

    // 设置点云字段（每个点包含x, y, z坐标）
    pointcloud.fields.resize(3);

    // x字段
    pointcloud.fields[0].name = "x";
    pointcloud.fields[0].offset = 0;
    pointcloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud.fields[0].count = 1;

    // y字段
    pointcloud.fields[1].name = "y";
    pointcloud.fields[1].offset = 4;
    pointcloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud.fields[1].count = 1;
        // RCLCPP_INFO(node_->get_logger(), "Published passability map with %zu points", cloud.size());
    // z字段
    pointcloud.fields[2].name = "z";
    pointcloud.fields[2].offset = 8;
    pointcloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloud.fields[2].count = 1;

    // 计算点步长（每个点占用的字节数）
    pointcloud.point_step = 12;  // 3个float32 = 12字节

    // 计算行步长
    pointcloud.row_step = pointcloud.point_step * pointcloud.width;

    // 是否为稠密点云（所有点都有效）
    pointcloud.is_dense = true;

    // 判断是否为大数据（这里简单判断）
    pointcloud.is_bigendian = false;

    // 分配数据存储空间
    pointcloud.data.resize(pointcloud.row_step * pointcloud.height);

    // 复制轨迹点到点云数据
    float* data_ptr = reinterpret_cast<float*>(&pointcloud.data[0]);

    for (size_t i = 0; i < traj.poses.size(); ++i) {
        const auto& pose = traj.poses[i];

        // 写入x, y, z坐标
        data_ptr[i * 3 + 0] = static_cast<float>(pose.position.x);
        data_ptr[i * 3 + 1] = static_cast<float>(pose.position.y);
        data_ptr[i * 3 + 2] = static_cast<float>(pose.position.z);
        std::cout<<"[TestMPPINode] Traj Point " << i << ": ("
                 << data_ptr[i * 3 + 0] << ", "
                 << data_ptr[i * 3 + 1] << ", "
                 << data_ptr[i * 3 + 2] << ")" <<std::endl;
    }

    return pointcloud;
}

//输出轨迹的函数
nav_msgs::msg::Path createStraightPath(
    double x0, double y0,
    double x1, double y1,
    int num_points = 51)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Clock().now();

    for (int i = 0; i < num_points; ++i) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = path.header.stamp;

        // 线性插值参数 t ∈ [0,1]
        double t = static_cast<double>(i) / (num_points - 1);

        // ⭐ 核心修改：起点 → 终点
        pose_stamped.pose.position.x = x0 + t * (x1 - x0);
        pose_stamped.pose.position.y = y0 + t * (y1 - y0);
        pose_stamped.pose.position.z = 0.0;

        // ⭐ 自动计算朝向（沿路径方向）
        double yaw = std::atan2(y1 - y0, x1 - x0);

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);

        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path.poses.push_back(pose_stamped);
    }

    return path;
}
// 设置path的函数


// 新增：一个持有 MPPIController 的 ROS2 节点
class TestMPPINode : public rclcpp::Node {
public:
    TestMPPINode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("test_mppi_node", options),
    grid_map_(std::make_shared<finenav::GridMap<float>>(
    finenav::Length{7.0, 7.0, 7.0}, 0.05)),
    grid_view_(*grid_map_,this)
    {
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_buffer_->setCreateTimerInterface(
            std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(),
                                                      this->get_node_timers_interface()));
        // QoS（goal 一般用 reliable）
        rclcpp::QoS goal_qos(10);
        goal_qos.reliable();

        // message_filter 订阅
        goal_sub_.subscribe(this, "/goal_pose", goal_qos.get_rmw_qos_profile());

        // TF filter（等 map <- base_link 可用）
        goal_tf_filter_ = std::make_shared<
            tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>(
            goal_sub_,
            *tf2_buffer_,
            "map",   // ⭐ 目标坐标系（关键）
            10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface(),
            100ms
        );

        // 注册回调
        goal_tf_filter_->registerCallback(&TestMPPINode::goalPoseCallback, this);
        traj_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "mppi_trajectory",   // 话题名（你可以改）
    10
);

        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        rclcpp::QoS qos = rclcpp::SensorDataQoS(); // 自动 BestEffort, Depth 10
        point_sub_.subscribe(this, "/cloud_for_lio", qos.get_rmw_qos_profile());

        tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
            point_sub_, *tf2_buffer_, "/odom", 10,
            this->get_node_logging_interface(), this->get_node_clock_interface(), 100ms);
        tf2_filter_->registerCallback(&TestMPPINode::pointcloudCallback, this);

        initController();
    }

    // 延迟初始化：在知道具体构造参数时调用
    void initController()
    {
        // 示例：如果 MPPIController 有合适的构造函数，按需传入参数并创建实例。
        // test_controller_.reset(new nav2_mppi_controller::MPPIController<SimpleGridMapView>(/* args... */, grid_view_));
        //
        // 目前保留为占位，不主动实例化以避免构造参数未知导致编译错误。
        // finenav::OBBCollisionModel collision_model({1.0, 1.0, 1.0});  // 示例碰撞模型参数
        std::cout<<"initController() called - 正在初始化 MPPIController..."<<std::endl;
        if (!test_controller_) {
            test_controller_ = std::make_unique<nav2_mppi_controller::MPPIController>();
        }

        finenav_mppi_controller::Params params; // 这里应根据实际需求设置参数,先做占位
        test_controller_-> configure(params);

        // finenav::PlanningContext<nav2_mppi_controller::MPPIController> ctx;
        // ctx.start_pose.position.x = 0.0;
        // ctx.start_pose.position.y = 0.0;
        // ctx.goal_pose.position.x = 6.0;
        // ctx.goal_pose.position.y = 0.0;
        //
        // finenav_msgs::msg::Trajectory output_traj;
        // nav_msgs::msg::Path path = createStraightPath();               //手动给一个path用作测试
        // test_controller_->setPlan(path);
        // test_controller_->plan(grid_view_, ctx, output_traj);
        // sensor_msgs::msg::PointCloud2 pcd = trajectoryToPointCloud(output_traj);
        // traj_pointcloud_pub_->publish(pcd);



        // RCLCPP_INFO(this->get_logger(), "initController() called - 请在此处根据实际构造签名构造 test_controller_");
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        const std::string sensor_frame = "base_lidar";     // TODO: 这些作为参数给用户
        const std::string base_frame = "base_link";
        const std::string world_frame = "map";

        geometry_msgs::msg::TransformStamped base_to_world_transform_stamped;
        geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
        try {
            base_to_world_transform_stamped = tf2_buffer_->lookupTransform(world_frame, base_frame, msg->header.stamp, 100ms);
            sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(world_frame, sensor_frame, msg->header.stamp, 100ms);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
            return;
        }

        finenav::Position base_posistion, sensor_position;
        base_posistion.x() = base_to_world_transform_stamped.transform.translation.x;
        base_posistion.y() = base_to_world_transform_stamped.transform.translation.y;
        base_posistion.z() = base_to_world_transform_stamped.transform.translation.z;

        grid_map_->moveTo(base_posistion);
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromROSMsg(*msg, pc);
        pcl_ros::transformPointCloud(pc, pc, sensor_to_world_transform_stamped);
        // for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        //     if(std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z)) {
        //         std::cout << *iter_x << ", "
        //                   << *iter_y << ", "
        //                   << *iter_z << std::endl;
        //     }
        // }
        for (const auto& p : pc) {
            if (numisfinite(p.x) && numisfinite(p.y) && numisfinite(p.z)) {
                finenav::Position pos{p.x, p.y,p.z};
                if (grid_map_->isInside(pos) ) {
                    grid_map_->atPosition(pos) = p.z;
                }
            }
        }
        // grid_view_.outputPassabilityImage();
        // grid_view_.outputCostmapImage();
        grid_view_.TerrainAnalysis(0.0);  // 这里假设机器人当前高度为0.0，实际使用中应根据机器人实际高度设置
        grid_view_.MapInflation();
        grid_view_.publishPassabilityMap();
        grid_view_.publishLocalMap();
    }
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 这里可以根据接收到的目标位姿触发规划过程
        std::cout << "Received goal pose: ("
                  << msg->pose.position.x << ", "
                  << msg->pose.position.y << ", "
                  << msg->pose.position.z << ")" << std::endl;

        const std::string base_frame = "base_link";
        const std::string world_frame = "map";
        geometry_msgs::msg::TransformStamped base_to_world_transform_stamped;

        try {
            base_to_world_transform_stamped = tf2_buffer_->lookupTransform(world_frame, base_frame, msg->header.stamp, 100ms);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
            return;
        }


        finenav::PlanningContext<nav2_mppi_controller::MPPIController> ctx;
        ctx.start_pose.position.x = base_to_world_transform_stamped.transform.translation.x;
        ctx.start_pose.position.y = base_to_world_transform_stamped.transform.translation.y;
        ctx.goal_pose.position.x = msg->pose.position.x;
        ctx.goal_pose.position.y = msg->pose.position.y;

        finenav_msgs::msg::Trajectory output_traj;
        nav_msgs::msg::Path path = createStraightPath(ctx.start_pose.position.x, ctx.start_pose.position.y, ctx.goal_pose.position.x,ctx.goal_pose.position.y,51);               //手动给一个path用作测试
        test_controller_->setPlan(path);
        // grid_view_.outputPassabilityImage("costmap.png");
        test_controller_->plan(grid_view_, ctx, output_traj);
        sensor_msgs::msg::PointCloud2 pcd = trajectoryToPointCloud(output_traj);
        traj_pointcloud_pub_->publish(pcd);
    }

private:
    // 持有 GridMap 与其视图
    std::shared_ptr<finenav::GridMap<float>>grid_map_;
    SimpleGridMapView grid_view_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traj_pointcloud_pub_;
    std::unique_ptr<nav2_mppi_controller::MPPIController> test_controller_;

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> goal_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>> goal_tf_filter_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf2_filter_;

};


