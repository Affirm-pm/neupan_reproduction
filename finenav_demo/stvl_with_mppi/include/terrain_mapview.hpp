// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"
#include "grid_map.hpp"
#include "terrain_analysis.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include <cmath>
using namespace mppi;
using std::chrono_literals::operator"" ms;



finenav::OBBCollisionModel collision_model({1.0, 1.0, 1.0});  //在实际使用中，尺寸应根据机器人实际尺寸设置应该有个tf回调函数实时更新位置
class SimpleGridMapView : public nav2_mppi_controller::IMapView {
public:
    explicit SimpleGridMapView(const finenav::GridMap<Voxel>& map, rclcpp::Node::SharedPtr node, std::shared_ptr<TerrainAnalyzer> terrain_analyzer) : map_(map),node_(node),terrain_analyzer_(terrain_analyzer) {
        if (node_) {
            local_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    "local_map", 10);
        }
    }


    // 是否将未知区域视为可通行（占位实现）
    /**
     * @brief 是否将未知区域视为可通行
     * @return 如果未知区域被视为可通行，则返回true；否则返回false
    */
    bool isTrackingUnknown() const override {
        return is_tracking_unknown_;
    }
    /**
     * @brief 是否考虑机器人足迹进行碰撞检测
     * @return 如果考虑足迹进行碰撞检测，则返回true；否则返回false
    */
    bool considerFootprint() const override {
        return consider_footprint_;
    }

    /**
     * @brief 在指定位姿下是否发生碰撞
     * @param x 机器人位姿的x坐标
     * @param y 机器人位姿的y坐标
     * @param theta 机器人位姿的朝向（弧度）
     * @return 如果在给定位姿下发生碰撞，则返回true；否则返回false
    */
    bool isCollision(float x, float y, float theta) const override {
        int pose_cost = getCost(finenav::Position3D{x, y, 0.0});
        finenav::Pose pose = createPose2D(x, y, theta);
        int possible_collision_cost = ComputeCost(collision_model.getOuterDiameter() / map_.getResolution());
        if (pose_cost >= possible_collision_cost) {
            return collision_model.checkCollision(pose, CollisionRule());
        }
        return false;
    }
    // 根据位置返回代价值（占位实现）

    /**
     * @brief 根据位置返回单点代价值
     * @param pos 位置，包含x、y、z坐标
     * @return 在给定位置处的代价值，通常是一个整数，表示该位置的通行成本。
    */
    int getCost(const finenav::Position3D& pos) const override {
        return terrain_analyzer_->getCost(pos, map_);
    }


    float getRadius() const override {
        return collision_model.getOuterDiameter();
    }
    /**message_filter
     * @brief 根据位姿返回位姿涵盖的最大代价值
     * @param x 机器人位姿的x坐标
     * @param y 机器人位姿的y坐标
     * @param theta 机器人位姿的朝向（弧度）
     * @return 在给定位姿处的代价值，通常是一个整数，表示该位姿涵盖的最大通行成本。该值可能考虑机器人足迹上的所有监测点，并返回其中的最大值，以反映该位姿的整体通行成本。
    */
    // 在给定位姿处使用足迹计算代价（占位实现）
    float costAtPose(
        float x, float y, float theta) const override
    {
        finenav::Pose pose = createPose2D(x, y, theta);
        return collision_model.checkCost(pose,[this](const finenav::Position3D& pos) {
            return this->getCost(pos);
        });
    }
    /**
     * @brief 返回地图的基准坐标系ID
     * @return 地图的基准坐标系ID，通常是一个字符串，例如"map"，表示该地图数据所使用的坐标系。
    */
    std::string getBaseFrameID() const override {
        return "map";
    }

    /**
     * @brief 进行地形分析，生成可通行性图
     * @param robot_pose_z 机器人当前位姿的z坐标，用于确定机器人位置来选择最近的地面高度
    */


    void publishLocalMap() {
        if (!node_ || !local_pub_ ) {
            return;
        }
        // 创建PCL点云
        pcl::PointCloud<pcl::PointXYZI> cloud;  // I 存储代价值
        cloud.header.frame_id = "map";
        // 遍历每个栅格
        for (int x = map_.getMinIndex().x(); x <= map_.getMaxIndex().x(); ++x) {
            for (int y = map_.getMinIndex().y(); y < map_.getMaxIndex().y(); ++y) {
                for(int z = map_.getMinIndex().z(); z < map_.getMaxIndex().z(); ++z) {
                    finenav::Position3D pos = map_.getPosition({x, y, z});
                        pcl::PointXYZI point;
                        // 计算世界坐标
                        point.x = pos.x();
                        point.y = pos.y();
                        point.z = map_.atPosition(pos).z_height;
                        cloud.push_back(point);
                }
            }
        }
        // 转换为ROS2消息并发布
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = node_->now();
        local_pub_->publish(cloud_msg);

    }

private:

    std::function<bool(const finenav::Position3D&)> CollisionRule() const {
        return [this](const finenav::Position3D& pos) {
            int pose_cost = this->getCost(pos);
            switch (static_cast<unsigned char>(pose_cost)) {
                case 254: // LETHAL_OBSTACLE
                    return true;
                case 255: // NO_INFORMATION
                    return is_tracking_unknown_ ? false : true;
            }
            return false;
        };
    }

    int ComputeCost(int distance) const {
        return terrain_analyzer_->ComputeCost(distance);
    }

    /**
     * @brief 根据二维位姿(x,y,theta)创建一个位姿，并将其转换为finenav::Pose格式，以便进行碰撞检测和代价计算
     * @param x 机器人位姿的x坐标
     * @param y 机器人位姿的y坐标
     * @param theta 机器人位姿的朝向（弧度）
     * @return 一个std::span<float>，包含(x,y)位置上沿z轴的一串栅格数据。
    */
    finenav::Pose createPose2D(double x, double y, double theta) const{
        finenav::Pose pose = finenav::Pose::Identity();
        // 设置旋转部分：绕Z轴旋转 theta
        pose.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        // 设置平移部分：x, y (z保持0)
        pose.translation() = Eigen::Vector3d(x, y, 0.0);
        return pose;
    }

    std::shared_ptr<TerrainAnalyzer> terrain_analyzer_;
    const finenav::GridMap<Voxel>& map_;
    double inscribed_radius_;
    bool is_tracking_unknown_{false};
    bool consider_footprint_{true};
    Eigen::ArrayXXf ground_array_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pub_;
};
