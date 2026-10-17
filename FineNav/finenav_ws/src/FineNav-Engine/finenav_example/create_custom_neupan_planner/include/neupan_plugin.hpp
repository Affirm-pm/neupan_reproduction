// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <chrono>
#include <mutex>
#include <memory>
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "finenav_core/plugin_map.hpp"
#include "finenav_core/plugin_planner.hpp"
#include "finenav_example/neupan_planner_params.hpp"
#include "finenav_example/neupan_cloud_map_params.hpp"
#include "finenav_example/neupan_world_view_params.hpp"
#include "neupan/neupan_planner.hpp"

namespace finenav::example {
using Steady = std::chrono::steady_clock;

class NeuPANCloudMap {
public:
    using ConfigType = neupan_cloud_map::Params;
    using DataType = sensor_msgs::msg::PointCloud2;
    void configure(const ConfigType& cfg) { topic_ = cfg.cloud_topic; }
    void selfManage(core::MapRuntime& runtime);
    const DataType* cloud() const { return cloud_.get(); }
    Steady::time_point received() const { return received_; }
private:
    std::string topic_;
    std::unique_ptr<DataType> cloud_;
    Steady::time_point received_{};
};

class NeuPANView {
public:
    virtual ~NeuPANView() = default;
    virtual bool ready() const = 0;
    virtual const neupan::Mat2X& points() const = 0;
};

class NeuPANPlanner {
public:
    using ConfigType = neupan_planner::Params;
    using IWorldView = NeuPANView;
    static constexpr core::IOProfile InputProfile{true, false, false, false};
    static constexpr core::IOProfile OutputProfile{false, true, false, false};
    using Context = core::PlanningContext<NeuPANPlanner>;
    using Result = std::optional<core::OutputTrajectoryT<NeuPANPlanner>>;
    void configure(const ConfigType& cfg);
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }
    void reset();
    Result plan(const Context& ctx, const IWorldView& view);
    // Exact rectangle distance, independent of neural distance prediction.
    bool collisionFree(const neupan::Vec3& state, const neupan::Vec2& command,
                       const neupan::Mat2X& points) const;
private:
    std::unique_ptr<neupan::NeuPANPlanner> planner_;
    std::vector<geometry_msgs::msg::Pose> reference_;
    double xy_tolerance_{0.08}, yaw_tolerance_{0.08}, margin_{0.05};
    rclcpp::Logger logger_{rclcpp::get_logger("NeuPANPlanner")};
};

class NeuPANWorldView : public NeuPANView {
public:
    using ConfigType = neupan_world_view::Params;
    NeuPANWorldView(rclcpp::Node::SharedPtr node, const NeuPANCloudMap& map)
        : node_(std::move(node)), map_(map) {}
    void configure(const ConfigType& cfg);
    void prepare(const NeuPANPlanner::Context& ctx);
    bool ready() const override { return ready_; }
    const neupan::Mat2X& points() const override { return points_; }
private:
    rclcpp::Node::SharedPtr node_;
    const NeuPANCloudMap& map_;
    ConfigType cfg_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_;
    Steady::time_point pose_received_{};
    bool ready_{false};
    neupan::Mat2X points_{2, 0};
};
}  // namespace finenav::example
