// Copyright (c) 2026. IWIN-FINS Lab, Shanghai Jiao Tong University.
#include "neupan_world_view.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {
NeuPanWorldView::NeuPanWorldView(rclcpp::Node::SharedPtr node, const DummyMap&)
    : node_(std::move(node)) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void NeuPanWorldView::configure(const ConfigType& cfg) {
    const bool source_changed = (!scan_sub_ && !cloud_sub_) ||
        cfg.points_topic != cfg_.points_topic || cfg.points_type != cfg_.points_type;
    cfg_ = cfg; // Worker-only: callbacks never access cfg_.
    valid_ = false;
    if (source_changed) installSubscription(cfg.points_topic, cfg.points_type);
}

void NeuPanWorldView::installSubscription(const std::string& topic, const std::string& type) {
    std::uint64_t generation;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        generation = ++generation_;
        scan_.reset(); cloud_.reset();
    }
    scan_sub_.reset(); cloud_sub_.reset();
    if (topic.empty()) return;
    if (type == "laser_scan") {
        scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            topic, rclcpp::SensorDataQoS(),
            [this, generation](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (generation != generation_) return;
                scan_ = std::move(msg); cloud_.reset();
                received_ = std::chrono::steady_clock::now();
            });
    } else if (type == "point_cloud2") {
        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this, generation](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (generation != generation_) return;
                cloud_ = std::move(msg); scan_.reset();
                received_ = std::chrono::steady_clock::now();
            });
    }
}

void NeuPanWorldView::prepare(const finenav::core::PlanningContext<NeuPanPlanner>& ctx) {
    valid_ = false;
    points_.clear(); collision_points_.clear();
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    std::chrono::steady_clock::time_point received;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        scan = scan_; cloud = cloud_; received = received_;
    }
    if (!scan && !cloud) return;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-received).count()
        > cfg_.points_timeout) return;
    const auto& header = scan ? scan->header : cloud->header;
    const rclcpp::Time stamp(header.stamp, node_->get_clock()->get_clock_type());
    const double age = (node_->now() - stamp).seconds();
    // Zero stamps request latest TF in ROS. Never silently accept that fallback.
    if (stamp.nanoseconds() == 0 || age < -0.05 || age > cfg_.points_timeout) return;
    const std::string source = cfg_.points_frame.empty() ? header.frame_id : cfg_.points_frame;
    if (source.empty()) return;
    try {
        if (cfg_.scan_range_min > cfg_.scan_range_max ||
            cfg_.scan_angle_min > cfg_.scan_angle_max || cfg_.min_height > cfg_.max_height)
            throw std::runtime_error("inverted obstacle filter bounds");
        if (scan) {
            if (!std::isfinite(scan->angle_min) || !std::isfinite(scan->angle_increment) ||
                !std::isfinite(scan->range_min) || !std::isfinite(scan->range_max) ||
                scan->range_min > scan->range_max || scan->ranges.empty())
                throw std::runtime_error("invalid scan metadata");
            // Keep every usable beam for collision checking. Downsampling is only
            // applied to the optimizer set after the world-frame conversion.
            for (std::size_t i=0; i<scan->ranges.size(); ++i) {
                const double r = scan->ranges[i];
                if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max) continue;
                const double a = cfg_.flip_angle ? scan->angle_max-i*scan->angle_increment
                                                 : scan->angle_min+i*scan->angle_increment;
                if (a < cfg_.scan_angle_min || a > cfg_.scan_angle_max) continue;
                collision_points_.push_back({r*std::cos(a), r*std::sin(a), 0.0});
            }
        } else {
            // Validate layout explicitly; an iterator<float> does not validate
            // FLOAT32 fields, row padding, endian order or truncated payloads.
            const auto offset = [&](const char* name, bool required) -> int {
                for (const auto& f : cloud->fields) if (f.name == name) {
                    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32 || f.count != 1 ||
                        f.offset + sizeof(float) > cloud->point_step)
                        throw std::runtime_error("cloud XYZ fields must be FLOAT32");
                    return static_cast<int>(f.offset);
                }
                if (required) throw std::runtime_error("cloud missing XY fields");
                return -1;
            };
            const int ox = offset("x", true), oy = offset("y", true), oz = offset("z", false);
            if (cloud->is_bigendian || cloud->row_step < std::uint64_t(cloud->width)*cloud->point_step ||
                cloud->data.size() < std::uint64_t(cloud->row_step)*cloud->height)
                throw std::runtime_error("invalid cloud layout");
            for (std::size_t row=0; row<cloud->height; ++row)
                for (std::size_t col=0; col<cloud->width; ++col) {
                    const auto* data=cloud->data.data()+row*cloud->row_step+col*cloud->point_step;
                    float x,y,z=0;
                    std::memcpy(&x,data+ox,4); std::memcpy(&y,data+oy,4);
                    if (oz>=0) std::memcpy(&z,data+oz,4);
                    if (std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z))
                        collision_points_.push_back({x,y,z});
                }
        }
        const auto& world = finenav::common::frames().world_fixed;
        if (source != world) {
            const auto tf = tf_buffer_->lookupTransform(world, source, stamp);
            const auto& t=tf.transform;
            const tf2::Matrix3x3 rotation(tf2::Quaternion(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w));
            const tf2::Vector3 translation(t.translation.x,t.translation.y,t.translation.z);
            for (auto& p:collision_points_) {
                const auto v=rotation*tf2::Vector3(p.x,p.y,p.z)+translation;
                p={v.x(),v.y(),v.z()};
            }
        }
        const bool has_z = cloud && std::any_of(cloud->fields.begin(),cloud->fields.end(),
            [](const auto& f){return f.name=="z";});
        const auto& robot=ctx.robot_state.pose.position;
        collision_points_.erase(std::remove_if(collision_points_.begin(),collision_points_.end(),
            [&](const auto& p){
                const double distance=std::hypot(p.x-robot.x,p.y-robot.y);
                return !std::isfinite(distance) || distance > cfg_.scan_range_max ||
                    (has_z && (p.z<cfg_.min_height || p.z>cfg_.max_height));
            }),collision_points_.end());
        const std::size_t stride=scan ? std::max<std::int64_t>(1,cfg_.scan_downsample) : 1;
        for(std::size_t i=0;i<collision_points_.size();i+=stride) {
            const auto& p=collision_points_[i];
            if (std::hypot(p.x-robot.x,p.y-robot.y)>=cfg_.scan_range_min) points_.push_back(p);
        }
        if (cfg_.max_points>0 && points_.size()>static_cast<std::size_t>(cfg_.max_points)) {
            const auto keep=static_cast<std::size_t>(cfg_.max_points);
            std::nth_element(points_.begin(),points_.begin()+keep,points_.end(),
                [&](const auto& a,const auto& b){return std::hypot(a.x-robot.x,a.y-robot.y)<
                                                           std::hypot(b.x-robot.x,b.y-robot.y);});
            points_.resize(keep);
        }
        valid_=true;
    } catch (const std::exception& e) {
        points_.clear(); collision_points_.clear();
        RCLCPP_WARN_THROTTLE(logger_,clock_,2000,"Invalid obstacle observation: %s",e.what());
    }
}
} // namespace finenav::example
FINENAV_REGISTER_WORLDVIEW(finenav::example::NeuPanWorldView, neupan_world_view,
    finenav::example::NeuPanPlanner, finenav::example::DummyMap)
