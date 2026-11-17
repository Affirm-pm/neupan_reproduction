#pragma once
#include <mutex>
#include <unordered_map>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "dummy_map.hpp"
#include "global_planner.hpp"
#include "neupan_world_view.hpp"
#include "finenav_example/global_grid_world_view_params.hpp"

namespace finenav::example {
class GlobalGridWorldView : public GlobalPlanner::IWorldView {
public:
    using ConfigType = global_grid_world_view::Params;
    GlobalGridWorldView(rclcpp::Node::SharedPtr node, const DummyMap& map)
        : node_(node), observations_(node, map) {}
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
    // Reuse the local view's timestamped TF, height/range filters and full cloud.
    NeuPanWorldView observations_;
    std::unique_ptr<neupan_world_view::ParamListener> observation_params_;
    struct LiveCell {
        double min_x, max_x, min_y, max_y;
        std::chrono::steady_clock::time_point stamp;
    };
    std::unordered_map<size_t, LiveCell> live_cells_;
    bool dirty_{true}, valid_{false};
    double start_x_{0}, start_y_{0};
    int cellIndex(const common::Position3D& p) const;
    std::vector<float> costs_;
};
}  // namespace finenav::example
