#pragma once
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "dummy_map.hpp"
#include "global_planner.hpp"
#include "finenav_example/global_grid_world_view_params.hpp"

namespace finenav::example {
class GlobalGridWorldView : public GlobalPlanner::IWorldView {
public:
    using ConfigType = global_grid_world_view::Params;
    GlobalGridWorldView(rclcpp::Node::SharedPtr node, const DummyMap&) : node_(node) {}
    void configure(const ConfigType& cfg);
    void prepare(const GlobalPlanner::Context&);
    bool isWalkable(const common::Position3D& p) const override;
    double getCost(const common::Position3D& p) const override;
    double getStepSize() const override { return valid_ ? grid_->info.resolution : 0.1; }
private:
    friend struct GlobalGridWorldViewTestAccess;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    std::mutex mutex_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr pending_, grid_;
    ConfigType cfg_;
    bool dirty_{true}, valid_{false};
    int cellIndex(const common::Position3D& p) const;
    std::vector<float> costs_;
};
}  // namespace finenav::example
