// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-2-3.
//
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_core/math_types.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"
#include "grid_map.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

using namespace mppi;


// 新增：将轨迹转换为点云的函数
sensor_msgs::msg::PointCloud2 trajectoryToPointCloud(const finenav_msgs::msg::Trajectory& traj) {
    sensor_msgs::msg::PointCloud2 pointcloud;

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
nav_msgs::msg::Path createStraightPath() {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";  // 设置坐标系
    path.header.stamp = rclcpp::Clock().now();  // 设置时间戳

    // 设置从 (0,0) 到 (5,5) 的直线路径
    int num_points = 51;  // 包含起点和终点的点数

    for (int i = 0; i < num_points; ++i) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = path.header.stamp;

        // 线性插值
        double t = static_cast<double>(i) / (num_points - 1);
        pose_stamped.pose.position.x = t * 6.0;  // 0 → 5
        pose_stamped.pose.position.y = t * 0.0;  // 0 → 5
        pose_stamped.pose.position.z = 0.0;

        // 设置朝向（朝向终点）
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;  // 无旋转

        path.poses.push_back(pose_stamped);
    }

    return path;
}
// 设置path的函数

class SimpleGridMapView : public nav2_mppi_controller::IMapView {
public:
    explicit SimpleGridMapView(finenav::GridMap<float>& map) : map_(map) {
        cv::Mat passability_image_ = cv::Mat::zeros(map_.getSize().x(), map_.getSize().y(), CV_8UC1);
    }

    // 是否将未知区域视为可通行（占位实现）
    bool isTrackingUnknown() const override {
        return is_tracking_unknown_;
    }

    bool considerFootprint() const override {
        return consider_footprint_;
    }

    // bool isCollision(
    //     float cost,
    //     float x, float y, float theta,
    //     bool consider_footprint) const override {
    //
    //     float pose_cost = cost;
    //
    //     double pose_cost = footprintCostAtPose(x, y, theta, getRobotFootprint());
    //
    //     switch (static_cast<unsigned char>(cost)) {
    //         // 使用具体数值替代 nav2_costmap_2d 宏
    //         // NO_INFORMATION = 255, LETHAL_OBSTACLE = 254, INSCRIBED_INFLATED_OBSTACLE = 253
    //         case 254: // LETHAL_OBSTACLE
    //             return true;
    //         case 253: // INSCRIBED_INFLATED_OBSTACLE
    //             return consider_footprint_ ? false : true;
    //         case 255: // NO_INFORMATION
    //             return is_tracking_unknown ? false : true;
    //     }
    //
    //     return false;
    // }
    // 根据位置返回代价值（占位实现）
    int getCost(const finenav::Position3D& pos) const override {
        return getCost(map_.getIndex(finenav::Position{pos.x(), pos.y(), pos.z()}));
    }

    int getCost(const finenav::Index& index) const override {
        return passability_image_.at<uchar>(index.y() + map_.getSize().y() / 2, index.x() + map_.getSize().x() / 2);  // TODO：将0/1的可通行性值转换为代价值
    }


    // 返回机器人当前足迹（占位实现，返回内部存储的空足迹）
    const Footprint & getRobotFootprint() const override {
        return footprint_;
    }

    // 在给定位姿处使用足迹计算代价（占位实现）
   float footprintCostAtPose(
       float x, float y, float theta,
       const Footprint & footprint) const override
   {
       double cos_th = cos(theta);
       double sin_th = sin(theta);
       Footprint oriented_footprint;
       oriented_footprint.reserve(footprint.size());
       std::pair<float, float> new_pt;
       for (unsigned int i = 0; i < footprint.size(); ++i) {
           new_pt.first = x + (footprint[i].first * cos_th - footprint[i].second * sin_th);
           new_pt.second = y + (footprint[i].first * sin_th + footprint[i].second * cos_th);
           oriented_footprint.push_back(new_pt);
       }
       return footprintCost(oriented_footprint);
   }

   double footprintCost(const Footprint & footprint) const {

       // 将第一个点转换为地图坐标
       if (map_.isInside(finenav::Position{footprint[0].first, footprint[0].second, 0.0})) {
           return 254;
       }

       double footprint_cost = 0.0;
       // 遍历足迹的每条边（线段）
       for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
           // 将下一个点转换为地图坐标
           if (map_.isInside(finenav::Position{footprint[i+1].first, footprint[i+1].second, 0.0})) {
               return 254;
           }

           // 计算这条线段上的最大成本
           footprint_cost = std::max(lineCost(footprint[i].first, footprint[i+1].first, footprint[i].second, footprint[i+1].second), footprint_cost);

           // 如果已经发现致命障碍，提前返回
           if (footprint_cost == 254) {
               return footprint_cost;
           }
       }

       // 连接最后一个点到第一个点（闭合多边形）
       return std::max(lineCost(footprint[3].first, footprint[0].first, footprint[3].second, footprint[0].second), footprint_cost);
   }

   double lineCost(double x0, double x1, double y0, double y1) const {
       // 这里可以实现 Bresenham 算法来遍历线段上的点，并返回最大代价
       // 目前占位实现，直接返回0
       std::vector<finenav::Index> grid_line_indices;
       map_.rayCast(finenav::Position{x0, y0, 0.0}, finenav::Position{x1, y1, 0.0}, grid_line_indices);
       double max_cost = 0.0;
       for (const auto& index : grid_line_indices) {
           if (map_.isInside(index)) {
               double cost = static_cast<double>(getCost(index));
               max_cost = std::max(max_cost, cost);
               if (max_cost == 254) {
                   break; // 提前退出，如果已经达到致命障碍#
               }
           }
       }
       return max_cost;

   }

    std::string getBaseFrameID() const override {
        return "map";
    }void printTrajectoryPoints(const finenav_msgs::msg::Trajectory& traj) {
    std::cout << "=== Trajectory Points ===" << std::endl;
    std::cout << "Total points: " << traj.poses.size() << std::endl;

    for (size_t i = 0; i < traj.poses.size(); ++i) {
        const auto& pose = traj.poses[i];
        std::cout << "Point " << i << ": "
                  << "(" << pose.position.x
                  << ", " << pose.position.y
                  << ", " << pose.position.z << ")" << std::endl;
    }
}
    void TerrainAnalysis(float robot_pose_z) {
        const float ROBOT_HEIGHT = 1.0;  // 根据您的机器人实际高度设置
        const float MAX_GRADIENT = 0.5;  // 根据您的需求设置最大坡度

        int sizeX_min = map_.getMinIndex().x();
        int sizeX_max = map_.getMaxIndex().x();
        int sizeY_min = map_.getMinIndex().y();
        int sizeY_max = map_.getMaxIndex().y();
        size_t size_x = map_.getSize().x();
        size_t size_y = map_.getSize().y();
        size_t half_size_x = size_x / 2;
        size_t half_size_y = size_y / 2;
        Eigen::ArrayXXf ground_array = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);

        for (int x = sizeX_min; x <= sizeX_max; ++x) {
            for (int y = sizeY_min; y <= sizeY_max; ++y) {
                std::span<float> voxels_along_z_indices = getVoxelsAlongZ(x, y);
                //这里可能要对获取的z值做一些处理，滤除掉特殊值之类

                float nearest_ground = std::numeric_limits<float>::max();
                if (!voxels_along_z_indices.empty()) {
                    for (size_t i = 0; i < voxels_along_z_indices.size() - 1; ++i) {
                        if(voxels_along_z_indices[i + 1] - voxels_along_z_indices[i] > ROBOT_HEIGHT) {
                            if (fabs(voxels_along_z_indices[i] - robot_pose_z) < fabs(nearest_ground - robot_pose_z)) {
                                nearest_ground = voxels_along_z_indices[i];
                            }
                        }
                    }
                    if (fabs(voxels_along_z_indices.back() - robot_pose_z) < fabs(nearest_ground - robot_pose_z)) {
                        nearest_ground = voxels_along_z_indices.back();
                    }
                }else{
                    nearest_ground = NAN;
                }
                ground_array(x + half_size_x, y + half_size_y) = nearest_ground;  // 记录ground NAN代表全空 其他值为有效值
            }
        }

        // TODO：对ground_array(x, y)进行中值滤波

        for (size_t x = 0; x < size_x; ++x) {
            for (size_t y = 0; y < size_y; ++y) {
                // 定义四个方向：上、右、下、左
                int dx[4] = {0, 1, 0, -1};
                int dy[4] = {1, 0, -1, 0};

                float terrain_value = 0;                    // 默认可通行
                float current_ground = ground_array(x, y);  //读取当前格子的ground高度
                // 检查当前格子本身
                if (!std::isnan(current_ground)) {  // 当前格子有ground
                    // 检查与周围四个格子的梯度
                    for (int i = 0; i < 4; ++i) {
                        int nx = x + dx[i];
                        int ny = y + dy[i];

                        // 检查是否越界
                        if (nx < 0 || nx > size_x - 1 || ny < 0 || ny > size_y - 1) {
                            continue;  // 忽略边界格子
                        }

                        float neighbor_ground = ground_array(nx, ny);  // 读取邻居格子的ground高度
                        // 处理特殊值
                        if (!std::isnan(neighbor_ground)) {            // 邻居格子有ground
                            // 计算高度差
                            float gradient = fabs(current_ground - neighbor_ground);
                            if (gradient > MAX_GRADIENT) {
                                terrain_value = 255;
                                break;
                            }
                        }
                    }
                }
                passability_image_.at<uchar>(y, x) = terrain_value;  // 记录可通行性
            }
        }
    }
    cv::Mat DistanceTransform() {
        cv::Mat passability_binary = 255 - passability_image_;  // 可通行区域为255，障碍物区域为0
        cv::Mat distance_transform;
        cv::distanceTransform(passability_binary, distance_transform, cv::DIST_L2, 3);
        return distance_transform;
    }

    void MapInflation() {
        cv::Mat distance_transform = DistanceTransform();
        for (int x = 0; x < passability_image_.cols; ++x) {
            for (int y = 0; y < passability_image_.rows; ++y) {
                if (passability_image_.at<uchar>(y, x) == 0) {  // 可通行区域
                    float distance = distance_transform.at<float>(y, x);
                    float cost;
                    if (distance * map_.getResolution() <= inscribed_radius_) {
                        cost = 253;
                    } else {
                        // make sure cost falls off by Euclidean distance
                        double factor =
                            exp(-1.0 * cost_scaling_factor_ * (distance * map_.getResolution() - inscribed_radius_));
                        cost = static_cast<unsigned char>((253 - 1) * factor);
                    }
                    passability_image_.at<uchar>(y, x) = static_cast<uchar>(cost);
                }
            }
        }
    }
    std::span<float> getVoxelsAlongZ(const int& x , const int& y) const {
        // 这里可以实现获取(x,y)位置上沿z轴的一串栅格数据，并返回最大代价
        // 目前占位实现，直接返回0
        std::span<float> voxels_along_z_indices;
        voxels_along_z_indices = map_.getVoxelsAlongZ(static_cast<int>(x), static_cast<int>(y));
        return voxels_along_z_indices;
    }


private:
    finenav::GridMap<float>& map_;
    Footprint footprint_;
    double inscribed_radius_;
    double cost_scaling_factor_;
    bool is_tracking_unknown_{false};
    bool consider_footprint_{true};
    cv::Mat passability_image_;  // 存储每个格子的可通行性值（0或1）
};

// 新增：一个持有 MPPIController 的 ROS2 节点
class TestMPPINode : public rclcpp::Node {
public:
    TestMPPINode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("test_mppi_node", options),

        grid_view_(grid_map_)
    {
        traj_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "trajectory_pointcloud",  // 话题名称，可自定义
    10                       // 队列长度
);
        RCLCPP_INFO(this->get_logger(), "TestMPPINode 创建完成，controller 尚未初始化。");
        // 如果需要立即初始化 controller，可以在此调用 initController()
        initController();
    }

    // 延迟初始化：在知道具体构造参数时调用
    void initController()
    {
        // 示例：如果 MPPIController 有合适的构造函数，按需传入参数并创建实例。
        // test_controller_.reset(new nav2_mppi_controller::MPPIController<SimpleGridMapView>(/* args... */, grid_view_));
        //
        // 目前保留为占位，不主动实例化以避免构造参数未知导致编译错误。
        if (!test_controller_) {
            test_controller_ = std::make_unique<nav2_mppi_controller::MPPIController>();
        }

        finenav_mppi_controller::Params params; // 这里应根据实际需求设置参数,先做占位
        test_controller_-> configure(params);

        finenav::PlanningContext<nav2_mppi_controller::MPPIController> ctx;
        ctx.start_pose.position.x = 0.0;
        ctx.start_pose.position.y = 0.0;
        ctx.goal_pose.position.x = 6.0;
        ctx.goal_pose.position.y = 0.0;

        finenav_msgs::msg::Trajectory output_traj;
        nav_msgs::msg::Path path = createStraightPath();               //手动给一个path用作测试
        test_controller_->setPlan(path);
        test_controller_->plan(grid_view_, ctx, output_traj);
        sensor_msgs::msg::PointCloud2 pcd = trajectoryToPointCloud(output_traj);
        traj_pointcloud_pub_->publish(pcd);



        // RCLCPP_INFO(this->get_logger(), "initController() called - 请在此处根据实际构造签名构造 test_controller_");
    }

private:
    // 持有 GridMap 与其视图
    finenav::GridMap<float> grid_map_;
    SimpleGridMapView grid_view_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traj_pointcloud_pub_;
    // 持有 MPPI controller（懒初始化）
    std::unique_ptr<nav2_mppi_controller::MPPIController> test_controller_;
};

