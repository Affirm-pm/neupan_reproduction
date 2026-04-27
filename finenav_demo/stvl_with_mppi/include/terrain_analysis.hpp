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
#include "voxel.hpp"


class TerrainAnalyzer {
public:
    /**
     * @brief 构造函数。map_server 仅用于查询地图初始尺寸，不存储。
     *        热路径中的所有地图访问通过 update()/getCost() 的参数直接传入，
     *        从而避免在 MapServer 已持锁期间再次竞争同一把锁。
     */
    explicit TerrainAnalyzer(
        const std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server,
        rclcpp::Node::SharedPtr node);

    /**
     * @brief 执行一次完整的地形分析与膨胀。
     *        必须在 MapServer 的 postUpdateHook 中调用，此时调用方已持有写锁，
     *        本函数直接使用传入的 map 引用，不会再次获取锁。
     * @param robot_pose_z 机器人当前 z 坐标
     * @param map          当前地图（由 postUpdateHook 直接传入）
     */
    void update(float robot_pose_z, const finenav::GridMap<Voxel>& map);

    /**
     * @brief 根据世界坐标查询代价值。
     *        调用方（SimpleGridMapView）已在 ControlLayer 持有读锁的上下文中运行，
     *        本函数不再内部获取锁。
     * @param pos  查询位置
     * @param map  当前地图（由 SimpleGridMapView 的 map_ 成员直接传入）
     */
    int getCost(const finenav::Position3D& pos, const finenav::GridMap<Voxel>& map) const;

    /**
     * @brief 将距离（栅格数）转换为代价值，使用缓存的分辨率，无需访问地图。
     */
    int ComputeCost(int distance) const;


private:
    void TerrainAnalysis(float robot_pose_z, const finenav::GridMap<Voxel>& map);
    void MapInflation();
    cv::Mat DistanceTransform();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passability_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr costmap_pub_;
    cv::Mat passability_image_;
    cv::Mat costmap_image_;
    double cost_scaling_factor_{1.0};
    double cached_resolution_{0.1};
};
