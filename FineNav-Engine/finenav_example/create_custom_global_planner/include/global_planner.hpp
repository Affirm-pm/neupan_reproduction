#pragma once

#include "finenav_route_planner/astar_path_search.hpp"
#include "finenav_example/finenav_global_planner_params.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

namespace finenav::example {

// Thin adapter: all search is performed by the unmodified official planner.
class GlobalPlanner {
public:
    using ConfigType = finenav_global_planner::Params;
    using IWorldView = astar::IWorldView;
    static constexpr core::IOProfile InputProfile{true, false, false, false};
    static constexpr core::IOProfile OutputProfile{true, false, false, false};
    using Context = core::PlanningContext<GlobalPlanner>;
    using Result = std::optional<core::OutputTrajectoryT<GlobalPlanner>>;
    void configure(const ConfigType& config);
    void reset() { planner_.reset(); }
    void setVisualizer(const viz::FineNavVisualizer& visualizer) { viz_ = visualizer; }
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }
    Result plan(const Context& ctx, const IWorldView& world);
private:
    void publishPath(const core::OutputTrajectoryT<GlobalPlanner>& path);
    AstarPathSearch planner_;
    viz::FineNavVisualizer viz_;
    rclcpp::Logger logger_{rclcpp::get_logger("GlobalPlanner")};
};
}  // namespace finenav::example
