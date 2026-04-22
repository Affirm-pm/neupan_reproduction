// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "grid_map.hpp"

#include <cmath>
#include <span>

#include <opencv2/opencv.hpp>

struct Voxel {
    float z_height = NAN;
    bool is_static = false;
    rclcpp::Time timestamp{};
};

class TerrainAnalyzer {
public:
    explicit TerrainAnalyzer(const finenav::GridMap<Voxel>& map, rclcpp::Node::SharedPtr node) : map_(map),node_(node) {
        passability_image_ = cv::Mat::zeros(map_.getSize().x(), map_.getSize().y(), CV_8UC1);
        costmap_image_ = cv::Mat::zeros(map_.getSize().x(), map_.getSize().y(), CV_8UC1);
        cost_scaling_factor_ = 0.5;  // 后续改成根据传入参数调整
        if (node_) {
            // 创建发布器map
            passability_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                "passability_map", 10);
        }
    }

    void update(float robot_pose_z) {
        TerrainAnalysis(robot_pose_z);
        MapInflation();
    }

    int getCost(const finenav::Position3D& pos) const  {
        if(map_.isInside(pos)) {
            finenav::Index index = map_.getIndex(pos);
            return costmap_image_.at<uchar>(index.y() + map_.getSize().y() / 2, index.x() + map_.getSize().x() / 2);
        }else {
            return 254;  // 越界视为不可通行,具体逻辑有待完善
        }

    }

    /**
    * @brief 根据距离计算代价值，使用指数衰减函数将距离转换为代价，以反映距离越近代价越高的关系
    * @param distance 距离值，通常是从距离变换图中获取的距离值，即距离最近障碍物的欧氏距离，单位为栅格数
    */
    int ComputeCost(int distance) const {
        const double factor = std::exp(-1.0 * cost_scaling_factor_ * (distance * map_.getResolution()));
        return static_cast<int>(253 * factor);
    }


private:
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
                std::span<const Voxel> voxels_along_z_indices = getVoxelsAlongZ(x, y);
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
        voxels_along_z_indices = map_.getVoxelsAlongZ(static_cast<int>(x), static_cast<int>(y));
        return voxels_along_z_indices;
    }

    const finenav::GridMap<Voxel>& map_;

    // TODO: 这些成员目前只是为了让头文件自洽可编译；后续由你的分析流程初始化/更新。
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passability_pub_;
    cv::Mat passability_image_;  // 存储每个格子的可通行性值（0或1）
    cv::Mat costmap_image_;  // 存储每个格子的可通行性值（0或1）
    double cost_scaling_factor_{1.0};
};

