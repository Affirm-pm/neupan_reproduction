// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_core/math_types.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"
#include "grid_map.hpp"

//#include <message_filters/subscriber.h>

//#include <tf2_ros/message_filter.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
//#include <tf2_ros/create_timer_ros.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl_ros/transforms.hpp>
//#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <opencv2/opencv.hpp>

//#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include <cmath>
using namespace mppi;
using std::chrono_literals::operator"" ms;

finenav::OBBCollisionModel collision_model({1.0, 1.0, 1.0});  //在实际使用中，尺寸应根据机器人实际尺寸设置应该有个tf回调函数实时更新位置
class SimpleGridMapView : public nav2_mppi_controller::IMapView {
public:
    explicit SimpleGridMapView(finenav::GridMap<float>& map, rclcpp::Node* node) : map_(map),node_(node) {
        passability_image_ = cv::Mat::zeros(map_.getSize().x(), map_.getSize().y(), CV_8UC1);
        costmap_image_ = cv::Mat::zeros(map_.getSize().x(), map_.getSize().y(), CV_8UC1);
        cost_scaling_factor_ = 0.5;
        if (node_) {
            // 创建发布器map
            passability_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                "passability_map", 10);
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
        if(map_.isInside(pos)) {
            finenav::Index index = map_.getIndex(pos);
            return costmap_image_.at<uchar>(index.y() + map_.getSize().y() / 2, index.x() + map_.getSize().x() / 2);
        }else {
            return 254;  // 越界视为不可通行,具体逻辑有待完善
        }

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
        Eigen::ArrayXXf ground_array_ = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);

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
                ground_array_(x + half_size_x, y + half_size_y) = nearest_ground;  // 记录ground NAN代表全空 其他值为有效值
            }
        }

        // TODO：对ground_array(x, y)进行中值滤波

        for (size_t x = 0; x < size_x; ++x) {
            for (size_t y = 0; y < size_y; ++y) {
                // 定义四个方向：上、右、下、左
                int dx[4] = {0, 1, 0, -1};
                int dy[4] = {1, 0, -1, 0};

                float terrain_value = 0;                    // 默认可通行
                float current_ground = ground_array_(x, y);  //读取当前格子的ground高度
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

                        float neighbor_ground = ground_array_(nx, ny);  // 读取邻居格子的ground高度
                        // 处理特殊值
                        if (!std::isnan(neighbor_ground)) {            // 邻居格子有ground
                            // 计算高度差
                            float gradient = fabs(current_ground - neighbor_ground);
                            if (gradient > MAX_GRADIENT) {
                                terrain_value = 254;
                                break;
                            }
                        }
                    }
                }
                passability_image_.at<uchar>(y, x) = terrain_value;  // 记录可通行性
            }
        }
    }

    /**
     * @brief 对地图进行膨胀处理，生成代价图
    */
    void MapInflation() {
        cv::Mat distance_transform = DistanceTransform();
        // outputCVImage(distance_transform);
        for (int x = 0; x < passability_image_.cols; ++x) {
            for (int y = 0; y < passability_image_.rows; ++y) {
                if (passability_image_.at<uchar>(y, x) == 0) {  // 可通行区域
                    float distance = distance_transform.at<float>(y, x);
                    int cost;
                    cost = ComputeCost(distance);
                    costmap_image_.at<uchar>(y, x) = static_cast<uchar>(cost);
                }else {
                    costmap_image_.at<uchar>(y, x) = 254;
                }
            }
        }
    }

    void outputPassabilityImage(const std::string& filename) {
        if (!passability_image_.empty()) {
            cv::imwrite(filename, passability_image_);
        }
    }

    void outputCVImage(cv::Mat &image) {
        if (!image.empty()) {
            for (int y = 0; y < image.rows; ++y) {
                for (int x = 0; x < image.cols; ++x) {
                    std::cout << static_cast<int>(image.at<uchar>(y, x)) << " ";
                }
                std::cout << std::endl;
            }
        }
    }

    void outputPassabilityImage() {
            outputCVImage(passability_image_);
    }

    void outputCostmapImage() {
        outputCVImage(costmap_image_);
    }

    void publishPassabilityMap() {
        if (!node_ || !passability_pub_ || passability_image_.empty()) {
            return;
        }

        // 创建PCL点云
        pcl::PointCloud<pcl::PointXYZI> cloud;  // I 存储代价值
        cloud.header.frame_id = "map";

        float resolution = map_.getResolution();
        size_t size_x = map_.getSize().x();
        size_t size_y = map_.getSize().y();

        // 遍历每个栅格
        for (int y = 0; y < passability_image_.rows; ++y) {
            for (int x = 0; x < passability_image_.cols; ++x) {
                uchar cost = passability_image_.at<uchar>(y, x);

                // 可以选择只发布非零代价的点，或者发布所有点
                if (cost ==254) {  // 只发布有代价的点，减少数据量
                    pcl::PointXYZI point;
                    // 计算世界坐标
                    point.x = (x - map_.getMaxIndex().x())* resolution + map_.getOrigin().x();  // 根据地图原点调整坐标
                    point.y = (y - map_.getMaxIndex().y())* resolution + map_.getOrigin().y();
                    point.z = 0.0;  // 可以设置为地面高度，如果需要的话
                    cloud.push_back(point);
                }
            }
        }

        // 转换为ROS2消息并发布
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = node_->now();
        passability_pub_->publish(cloud_msg);


        // RCLCPP_INFO(node_->get_logger(), "Published passability map with %zu points", cloud.size());
    }

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
                        point.z = map_.atPosition(pos);
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
    /**
     * @brief 对地图中的无源栅格进行距离变换，生成距离图，以便后续进行膨胀处理
    */
    cv::Mat DistanceTransform() {
        cv::Mat passability_binary = 254 - passability_image_;  // 可通行区域为255，障碍物区域为0
        cv::Mat distance_transform;
        cv::distanceTransform(passability_binary, distance_transform, cv::DIST_L2, 3);
        return distance_transform;
    }
    /**
     * @brief 根据距离计算代价值，使用指数衰减函数将距离转换为代价，以反映距离越近代价越高的关系
     * @param distance 距离值，通常是从距离变换图中获取的距离值，即距离最近障碍物的欧氏距离，单位为栅格数
    */
    int ComputeCost(int distance) const{
        double factor = exp(-1.0 * cost_scaling_factor_ * (distance * map_.getResolution()));
        return 253 * factor;
    }
    /**
     * @brief 获取(x,y)位置上沿z轴的一串栅格数据，用于地形分析
     * @return 一个std::span<float>，包含(x,y)位置上沿z轴的一串栅格数据。
    */
    std::span<float> getVoxelsAlongZ(const int& x , const int& y) const {
        std::span<float> voxels_along_z_indices;
        voxels_along_z_indices = map_.getVoxelsAlongZ(static_cast<int>(x), static_cast<int>(y));
        return voxels_along_z_indices;
    }

    /**
     * @brief 获取(x,y)位置上沿z轴的一串栅格数据，用于地形分析
     * @return 一个std::span<float>，包含(x,y)位置上沿z轴的一串栅格数据。
    */
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




    finenav::GridMap<float>& map_;
    double inscribed_radius_;
    double cost_scaling_factor_;
    bool is_tracking_unknown_{false};
    bool consider_footprint_{true};
    Eigen::ArrayXXf ground_array_;
    cv::Mat passability_image_;  // 存储每个格子的可通行性值（0或1）
    cv::Mat costmap_image_;  // 存储每个格子的可通行性值（0或1）
    rclcpp::Node* node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passability_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pub_;
};
