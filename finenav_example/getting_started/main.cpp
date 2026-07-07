// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Getting started: assemble a complete navigation stack using FineNavEngine.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "finenav_engine/finenav_engine.hpp"
#include "finenav_core/plan/planner_set.hpp" // TODO: 这个文件要搞掉

#include "dummy_map.hpp"
#include "dummy_planner.hpp"

using namespace finenav::example;
using namespace finenav::core;

class ASimpleMapView : public DummyMapView {
public:
    ASimpleMapView(const DummyMap& map_ptr) : map_(map_ptr) {}
    bool isWalkable(const Position3D&) const override {
        return !map_.isOccupied(Position3D());
    }
private:
    const DummyMap& map_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hello_finenav");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    // 1. Create map
    auto map = engine->createMapServer<DummyMap>("my_map");

    // 2. Create layer
    using Planners = PlannerSet<DummyPlanner>;
    auto layer = engine->createControlLayer<Planners>("my_layer", ContinuousPolicy{10.0});    // TODO: 这里就不应该再允许输入参数了

    // 3. Bind map to layer
    layer->bind_map_view(map, [](const DummyMap& map) {
        return std::make_shared<ASimpleMapView>(map);
    });

    // 5. Post-plan: publish cmd_vel // TODO： 不止支持cmd_vel
    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);
    layer->set_on_post_plan(
        [cmd_vel_pub](finenav::core::PostPlanContext& ctx) -> bool {
            if (ctx.is_success && !ctx.result_traj.twists.empty()) {
                cmd_vel_pub->publish(ctx.result_traj.twists[0]);
            }
            return ctx.is_success;
        });

    // 6. Connect pipeline
    engine->connectPipeline({"line_path", "plan_result"}, layer);

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine ready.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
