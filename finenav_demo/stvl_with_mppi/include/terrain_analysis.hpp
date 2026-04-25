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
    /**
     * @brief 构造函数。map_server 仅用于查询地图初始尺寸，不存储。
     *        热路径中的所有地图访问通过 update()/getCost() 的参数直接传入，
     *        从而避免在 MapServer 已持锁期间再次竞争同一把锁。
     */
    explicit TerrainAnalyzer(
        const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server,
        rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        // 仅在构造阶段查询一次地图尺寸，之后不再持有 map_server 引用。
        const auto map_ro = map_server->getLockedReadView();
        passability_image_ = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
        costmap_image_     = cv::Mat::zeros(map_ro->getSize().x(), map_ro->getSize().y(), CV_8UC1);
        cached_resolution_ = map_ro->getResolution();
        cost_scaling_factor_ = 0.5;
        if (node_) {
            passability_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("passability_map", 10);
            costmap_pub_     = node_->create_publisher<sensor_msgs::msg::PointCloud2>("costmap_cloud", 10);
        }
    }

    /**
     * @brief 执行一次完整的地形分析与膨胀。
     *        必须在 MapServer 的 postUpdateHook 中调用，此时调用方已持有写锁，
     *        本函数直接使用传入的 map 引用，不会再次获取锁。
     * @param robot_pose_z 机器人当前 z 坐标
     * @param map          当前地图（由 postUpdateHook 直接传入）
     */
    void update(float robot_pose_z, const finenav::GridMap<Voxel>& map) {
        cached_resolution_ = map.getResolution();  // 刷新缓存分辨率
        TerrainAnalysis(robot_pose_z, map);
        MapInflation();
        publishCostmapPointCloud(map);
    }

    /**
     * @brief 根据世界坐标查询代价值。
     *        调用方（SimpleGridMapView）已在 ControlLayer 持有读锁的上下文中运行，
     *        本函数不再内部获取锁。
     * @param pos  查询位置
     * @param map  当前地图（由 SimpleGridMapView 的 map_ 成员直接传入）
     */
    int getCost(const finenav::Position3D& pos, const finenav::GridMap<Voxel>& map) const {
        if (map.isInside(pos)) {
            finenav::Index index = map.getIndex(pos);
            return costmap_image_.at<uchar>(
                index.y() + map.getSize().y() / 2,
                index.x() + map.getSize().x() / 2);
        }
        return 254;  // 越界视为不可通行
    }

    /**
     * @brief 将距离（栅格数）转换为代价值，使用缓存的分辨率，无需访问地图。
     */
    int ComputeCost(int distance) const {
        const double factor = std::exp(-1.0 * cost_scaling_factor_ * (distance * cached_resolution_));
        return static_cast<int>(253 * factor);
    }

    /**
     * @brief 以彩色点云形式发布当前代价地图。
     * @param map  当前地图（由 update() 传入，不另外加锁）
     */
    void publishCostmapPointCloud(const finenav::GridMap<Voxel>& map, float z_layer = 0.0f) {
        if (!node_ || !costmap_pub_) return;

        const int min_x = map.getMinIndex().x();
        const int max_x = map.getMaxIndex().x();
        const int min_y = map.getMinIndex().y();
        const int max_y = map.getMaxIndex().y();

        const size_t nx   = static_cast<size_t>(max_x - min_x + 1);
        const size_t ny   = static_cast<size_t>(max_y - min_y + 1);
        const size_t npts = nx * ny;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp    = node_->now();
        cloud.header.frame_id = "map";
        cloud.height          = 1;
        cloud.width           = static_cast<uint32_t>(npts);
        cloud.is_dense        = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
            4,
            "x",   1, sensor_msgs::msg::PointField::FLOAT32,
            "y",   1, sensor_msgs::msg::PointField::FLOAT32,
            "z",   1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(npts);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_rgb(cloud, "rgb");

        auto packRGB = [](uint8_t r, uint8_t g, uint8_t b) -> float {
            const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                                 (static_cast<uint32_t>(g) << 8)  |
                                 (static_cast<uint32_t>(b));
            float out;
            std::memcpy(&out, &rgb, sizeof(uint32_t));
            return out;
        };

        constexpr uint8_t kBlueMin = 40;

        for (int ix = min_x; ix <= max_x; ++ix) {
            for (int iy = min_y; iy <= max_y; ++iy) {
                const finenav::Position3D pos =
                    map.getPosition(finenav::Index{ix, iy, map.getMinIndex().z()});

                const int row  = iy + static_cast<int>(map.getSize().y() / 2);
                const int col  = ix + static_cast<int>(map.getSize().x() / 2);
                const int cost = static_cast<int>(costmap_image_.at<uchar>(row, col));

                uint8_t r = 0, g = 0, b = 0;
                if (cost >= 254) {
                    r = 0; g = 0; b = 0;
                } else {
                    const float t = std::clamp(static_cast<float>(cost) / 253.0f, 0.0f, 1.0f);
                    b = static_cast<uint8_t>(kBlueMin + t * (255.0f - kBlueMin));
                }

                *iter_x   = static_cast<float>(pos.x());
                *iter_y   = static_cast<float>(pos.y());
                *iter_z   = z_layer;
                *iter_rgb = packRGB(r, g, b);
                ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
            }
        }
        costmap_pub_->publish(cloud);
    }


private:
    void TerrainAnalysis(float robot_pose_z, const finenav::GridMap<Voxel>& map) {
        const float ROBOT_HEIGHT = 1.0;
        const float MAX_GRADIENT = 0.5;

        int sizeX_min = map.getMinIndex().x();
        int sizeX_max = map.getMaxIndex().x();
        int sizeY_min = map.getMinIndex().y();
        int sizeY_max = map.getMaxIndex().y();
        size_t size_x      = map.getSize().x();
        size_t size_y      = map.getSize().y();
        size_t half_size_x = size_x / 2;
        size_t half_size_y = size_y / 2;

        Eigen::ArrayXXf ground_array_ = Eigen::ArrayXXf::Constant(size_x, size_y, NAN);

        for (int x = sizeX_min; x <= sizeX_max; ++x) {
            for (int y = sizeY_min; y <= sizeY_max; ++y) {
                std::span<const Voxel> voxels =
                    map.getVoxelsAlongZ(static_cast<int>(x), static_cast<int>(y));

                float nearest_ground = std::numeric_limits<float>::max();
                if (!voxels.empty()) {
                    for (size_t i = 0; i < voxels.size() - 1; ++i) {
                        if (voxels[i + 1].z_height - voxels[i].z_height > ROBOT_HEIGHT) {
                            if (fabs(voxels[i].z_height - robot_pose_z) <
                                fabs(nearest_ground - robot_pose_z)) {
                                nearest_ground = voxels[i].z_height;
                            }
                        }
                    }
                    if (fabs(voxels.back().z_height - robot_pose_z) <
                        fabs(nearest_ground - robot_pose_z)) {
                        nearest_ground = voxels.back().z_height;
                    }
                } else {
                    nearest_ground = NAN;
                }
                ground_array_(x + half_size_x, y + half_size_y) = nearest_ground;
            }
        }

        for (size_t x = 0; x < size_x; ++x) {
            for (size_t y = 0; y < size_y; ++y) {
                int dx[4] = {0, 1, 0, -1};
                int dy[4] = {1, 0, -1, 0};

                float terrain_value  = 0;
                float current_ground = ground_array_(x, y);
                if (!std::isnan(current_ground)) {
                    for (int i = 0; i < 4; ++i) {
                        int nx = x + dx[i];
                        int ny = y + dy[i];
                        if (nx < 0 || nx > static_cast<int>(size_x) - 1 ||
                            ny < 0 || ny > static_cast<int>(size_y) - 1) continue;
                        float neighbor_ground = ground_array_(nx, ny);
                        if (!std::isnan(neighbor_ground)) {
                            if (fabs(current_ground - neighbor_ground) > MAX_GRADIENT) {
                                terrain_value = 254;
                                break;
                            }
                        }
                    }
                }
                passability_image_.at<uchar>(y, x) = terrain_value;
            }
        }
    }

    void MapInflation() {
        cv::Mat distance_transform = DistanceTransform();
        for (int x = 0; x < passability_image_.cols; ++x) {
            for (int y = 0; y < passability_image_.rows; ++y) {
                if (passability_image_.at<uchar>(y, x) == 0) {
                    float distance = distance_transform.at<float>(y, x);
                    costmap_image_.at<uchar>(y, x) = static_cast<uchar>(ComputeCost(distance));
                } else {
                    costmap_image_.at<uchar>(y, x) = 254;
                }
            }
        }
    }

    cv::Mat DistanceTransform() {
        cv::Mat passability_binary = 254 - passability_image_;
        cv::Mat distance_transform;
        cv::distanceTransform(passability_binary, distance_transform, cv::DIST_L2, 3);
        return distance_transform;
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passability_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr costmap_pub_;
    cv::Mat passability_image_;
    cv::Mat costmap_image_;
    double cost_scaling_factor_{1.0};
    double cached_resolution_{0.1};  // 由 update() 每帧刷新，构造时初始化
};

