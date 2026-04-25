// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "grid_map.hpp"
#include "finenav_core/map/map_server.hpp"
#include <cmath>
#include <span>
#include <algorithm>
#include <cstring>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

struct Voxel {
    float z_height = NAN;
    bool is_static = false;
    rclcpp::Time timestamp{};
};

class TerrainAnalyzer {
public:
    explicit TerrainAnalyzer(const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server, rclcpp::Node::SharedPtr node) : map_server_(map_server),node_(node) {
        // One locked view for init-time map size query.
        const auto map_ro = map_server_->getLockedReadView();
        passability_image_ = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
        costmap_image_ = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
        cost_scaling_factor_ = 0.5;  // 后续改成根据传入参数调整
        if (node_) {
            // 创建发布器map
            passability_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                "passability_map", 10);

            // 代价地图（彩色点云）发布器
            costmap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                "costmap_cloud", 10);
        }
    }

    void update(float robot_pose_z) {
        TerrainAnalysis(robot_pose_z);
        MapInflation();

        // 发布彩色代价地图点云（如果需要可改成外部触发/限频）
        publishCostmapPointCloud();
    }

    int getCost(const finenav::Position3D& pos) const  {
        const auto map_ro = map_server_->getLockedReadView();
        if(map_ro->isInside(pos)) {
            finenav::Index index = map_ro->getIndex(pos);
            return costmap_image_.at<uchar>(index.y() + map_ro->getSize().y() / 2, index.x() + map_ro->getSize().x() / 2);
        }else {
            return 254;  // 越界视为不可通行,具体逻辑有待完善
        }

    }

    /**
    * @brief 根据距离计算代价值，使用指数衰减函数将距离转换为代价，以反映距离越近代价越高的关系
    * @param distance 距离值，通常是从距离变换图中获取的距离值，即距离最近障碍物的欧氏距离，单位为栅格数
    */
    int ComputeCost(int distance) const {
        const auto map_ro = map_server_->getLockedReadView();
        const double factor = std::exp(-1.0 * cost_scaling_factor_ * (distance * map_ro->getResolution()));
        return static_cast<int>(253 * factor);
    }

    /**
     * @brief 以彩色点云形式发布当前代价地图。
     * 颜色规则：
     *   - cost == 254 : 黑色（不可通行）
     *   - cost 由低到高 : 蓝色由浅到深
     */
    void publishCostmapPointCloud(float z_layer = 0.0f) {
        if (!node_ || !costmap_pub_) {
            return;
        }

        const auto map_ro = map_server_->getLockedReadView();

        const int min_x = map_ro->getMinIndex().x();
        const int max_x = map_ro->getMaxIndex().x();
        const int min_y = map_ro->getMinIndex().y();
        const int max_y = map_ro->getMaxIndex().y();

        const size_t nx = static_cast<size_t>(max_x - min_x + 1);
        const size_t ny = static_cast<size_t>(max_y - min_y + 1);
        const size_t npts = nx * ny;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = node_->now();
        cloud.header.frame_id = "map";
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(npts);
        cloud.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(npts);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud, "rgb");

        auto packRGB = [](uint8_t r, uint8_t g, uint8_t b) -> float {
            const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                                 (static_cast<uint32_t>(g) << 8) |
                                 (static_cast<uint32_t>(b));
            float out;
            std::memcpy(&out, &rgb, sizeof(uint32_t));
            return out;
        };

        // 蓝色最浅的下限（避免低代价几乎不可见）
        constexpr uint8_t kBlueMin = 40;

        for (int ix = min_x; ix <= max_x; ++ix) {
            for (int iy = min_y; iy <= max_y; ++iy) {
                // 获取该 (ix,iy) 对应的世界坐标；z 固定为一张 2D 图层
                const finenav::Position3D pos = map_ro->getPosition(finenav::Index{ix, iy, map_ro->getMinIndex().z()});

                const int row = iy + static_cast<int>(map_ro->getSize().y() / 2);
                const int col = ix + static_cast<int>(map_ro->getSize().x() / 2);

                const int cost = static_cast<int>(costmap_image_.at<uchar>(row, col));

                uint8_t r = 0, g = 0, b = 0;
                if (cost >= 254) {
                    // 不可通行：黑色
                    r = 0; g = 0; b = 0;
                } else {
                    // 0~253: 蓝色由浅到深
                    const float t = std::clamp(static_cast<float>(cost) / 253.0f, 0.0f, 1.0f);
                    b = static_cast<uint8_t>(kBlueMin + t * (255.0f - kBlueMin));
                    r = 0;
                    g = 0;
                }

                *iter_x = static_cast<float>(pos.x());
                *iter_y = static_cast<float>(pos.y());
                *iter_z = z_layer;
                *iter_rgb = packRGB(r, g, b);

                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_rgb;
            }
        }

        costmap_pub_->publish(cloud);
    }


private:
    void TerrainAnalysis(float robot_pose_z) {
        const float ROBOT_HEIGHT = 1.0;  // 根据您的机器人实际高度设置
        const float MAX_GRADIENT = 0.5;  // 根据您的需求设置最大坡度

        const auto map_ro = map_server_->getLockedReadView();

        int sizeX_min = map_ro->getMinIndex().x();
        int sizeX_max = map_ro->getMaxIndex().x();
        int sizeY_min = map_ro->getMinIndex().y();
        int sizeY_max = map_ro->getMaxIndex().y();
        size_t size_x = map_ro->getSize().x();
        size_t size_y = map_ro->getSize().y();
        size_t half_size_x = size_x / 2;
        size_t half_size_y = size_y / 2;
        Eigen::ArrayXXf ground_array_ = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);

        for (int x = sizeX_min; x <= sizeX_max; ++x) {
            for (int y = sizeY_min; y <= sizeY_max; ++y) {
                std::span<const Voxel> voxels_along_z_indices = map_ro->getVoxelsAlongZ(static_cast<int>(x), static_cast<int>(y));
                //这里可能要对获取的z值做一些处理，滤除掉特殊值之类

                float nearest_ground = std::numeric_limits<float>::max();
                if (!voxels_along_z_indices.empty()) {
                    for (size_t i = 0; i < voxels_along_z_indices.size() - 1; ++i) {
                        if(voxels_along_z_indices[i + 1].z_height - voxels_along_z_indices[i].z_height > ROBOT_HEIGHT) {
                            if (fabs(voxels_along_z_indices[i].z_height - robot_pose_z) < fabs(nearest_ground - robot_pose_z)) {
                                nearest_ground = voxels_along_z_indices[i].z_height;
                            }
                        }
                    }
                    if (fabs(voxels_along_z_indices.back().z_height - robot_pose_z) < fabs(nearest_ground - robot_pose_z)) {
                        nearest_ground = voxels_along_z_indices.back().z_height;
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
     * @brief 获取(x,y)位置上沿z轴的一串栅格数据，用于地形分析
     * @return 一个std::span<Voxel>，包含(x,y)位置上沿z轴的一串栅格数据。
     */
    std::span<const Voxel> getVoxelsAlongZ(const int& x, const int& y) const {
        std::span<const Voxel> voxels_along_z_indices;
        voxels_along_z_indices = map_server_->getLockedReadView()->getVoxelsAlongZ(static_cast<int>(x), static_cast<int>(y));
        return voxels_along_z_indices;
    }

    const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server_;
    // TODO: 这些成员目前只是为了让头文件自洽可编译；后续由你的分析流程初始化/更新。
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passability_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr costmap_pub_;
    cv::Mat passability_image_;  // 存储每个格子的可通行性值（0或1）
    cv::Mat costmap_image_;  // 存储每个格子的可通行性值（0或1）
    double cost_scaling_factor_{1.0};
};

