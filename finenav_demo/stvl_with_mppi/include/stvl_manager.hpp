// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef STVL_MANAGER_HPP_
#define STVL_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <unordered_set>
#include <Eigen/Core>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include "grid_map.hpp" // TODO: resolve anti-pattern export
#include "voxel.hpp"

namespace finenav {

struct Pointcloud {
    float x;
    float y;
    float z;
};

struct PositionHash {
    std::size_t operator()(const Position & p) const noexcept {
        std::size_t hx = std::hash<double>{}(p.x());
        std::size_t hy = std::hash<double>{}(p.y());
        std::size_t hz = std::hash<double>{}(p.z());
        auto hash_combine = [](std::size_t seed, std::size_t v) {
            seed ^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            return seed;
        };
        std::size_t seed = 0;
        seed = hash_combine(seed, hx);
        seed = hash_combine(seed, hy);
        seed = hash_combine(seed, hz);
        return seed;
    }
};

struct PositionEqual {
    bool operator()(const Position & a, const Position & b) const noexcept {
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

using DynamicPosSet = std::unordered_set<Position, PositionHash, PositionEqual>;

class StvlManager {
public:
    StvlManager(rclcpp::Node::SharedPtr node);

    // Initialize mapping components
    void InsertPointCloud(const Pointcloud & msg, GridMap<Voxel> & map);

    void PruneExpiredCells(finenav::GridMap<Voxel> & map);
private:
    bool isInFrustum(const Eigen::Matrix4d & T_ws, const Position & p_world,
                   double min_range, double max_range,
                   double min_angle, double max_angle) const;

    rclcpp::Node::SharedPtr node_;

    DynamicPosSet dynamic_cells_;
    double decay_time_sec_ = 1000000.0;
    double decay_time_fov_sec_ = 1000000.0;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_filter_;


};

} // namespace finenav

#endif // STVL_MANAGER_HPP_

