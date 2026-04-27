// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "stvl_manager.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/create_timer_ros.h>

using namespace finenav;
using std::chrono_literals::operator""ms;
static bool numisfinite(float x) {
    return std::isfinite(x);
}

StvlManager::StvlManager(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_buffer_->setCreateTimerInterface(
        std::make_shared<tf2_ros::CreateTimerROS>(
            node_->get_node_base_interface(),
            node_->get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool StvlManager::isInFrustum(const Eigen::Matrix4d & T_ws, const Position & p_world,
                              double min_range, double max_range,
                              double min_angle, double max_angle) const {
    const Eigen::Matrix3d R_ws = T_ws.topLeftCorner<3, 3>();
    const Eigen::Vector3d origin = T_ws.topRightCorner<3, 1>();
    const Eigen::Vector3d v_world = p_world - origin;

    const double r = v_world.norm();
    if (r <= 0.0 || r < min_range || r > max_range) {
        return false;
    }

    const Eigen::Vector3d v_sensor = R_ws.transpose() * v_world;
    const double norm = v_sensor.norm();
    if (norm <= 0.0) {
        return false;
    }

    const double cos_theta = std::clamp(v_sensor.z() / norm, -1.0, 1.0);
    const double theta = std::acos(cos_theta);
    return theta >= min_angle && theta <= max_angle;
}

void StvlManager::PruneExpiredCells(GridMap<Voxel> & map)  {
    const rclcpp::Time now = node_->get_clock()->now();

    Eigen::Matrix4d T_ws = Eigen::Matrix4d::Identity();
    try {
        const auto tf_stamped = tf_buffer_->lookupTransform("map", "base_lidar", tf2::TimePointZero);
        const Eigen::Isometry3d iso = tf2::transformToEigen(tf_stamped.transform);
        T_ws = iso.matrix();
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "TF lookup failed in pre-update hook: %s", ex.what());
    }

    constexpr double min_range = 0.0;
    constexpr double max_range = 50.0;
    constexpr double min_angle = 0.0;
    constexpr double max_angle = M_PI / 3.0;        //后续改成成员变量，并提供接口动态调整

    for (auto it = dynamic_cells_.begin(); it != dynamic_cells_.end();) {
        const Position & pos = *it;

        if (!map.isInside(pos)) {
            it = dynamic_cells_.erase(it);
            continue;
        }

        Voxel & cell = map.atPosition(pos);
        if (cell.timestamp.nanoseconds() == 0) {             //TODO：按照逻辑不应该出现，但实际上确实有哈希表里的索引对应的是time{},存在隐患，后续排查
            it = dynamic_cells_.erase(it);
            continue;
        }

        const double age_sec = (now - cell.timestamp).seconds();

        if (age_sec > decay_time_sec_) {
            cell = Voxel{NAN, false, rclcpp::Time{}};
            it = dynamic_cells_.erase(it);
            continue;
        }

        const bool in_fov = isInFrustum(T_ws, pos, min_range, max_range, min_angle, max_angle);
        const double active_decay = in_fov ? decay_time_fov_sec_ : decay_time_sec_;

        if (age_sec > active_decay) {
            cell = Voxel{NAN, false, rclcpp::Time{}};
            it = dynamic_cells_.erase(it);
        } else {
            ++it;
        }
    }

}

void StvlManager::InsertPointCloud(const Pointcloud & msg, GridMap<Voxel> & map){
    const rclcpp::Time now = node_->get_clock()->now();
    const Position hit_pos(msg.x, msg.y, msg.z);
    if (!map.isInside(hit_pos)) {
        return;
    }

    const Index idx = map.getIndex(hit_pos);
    Voxel & cell = map.at(idx);
    if (!cell.is_static) {
        cell.z_height = msg.z;
        cell.timestamp = now;
        dynamic_cells_.insert(map.getPosition(idx));
    }
}