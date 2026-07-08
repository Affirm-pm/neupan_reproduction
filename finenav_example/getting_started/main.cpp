// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Getting started: assemble a complete navigation stack using FineNavEngine.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "finenav_engine/finenav_engine.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include "dummy_map.hpp"
#include "dummy_planner.hpp"

using namespace finenav::example;
using namespace finenav::core; // TODO: 看看能不能只用finenav而不用finenav::core

/**
 *  @brief Define a MapView, which determine the rule for planner
 *  @details In this example, the rule is quite simple: the planner only considers the map occupied if the position is occupied in the map.
 */
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

    // 1. Create MapServer — engine stores internally
    engine->createMapServer<DummyMap>("my_map");

    // 2. Create ControlLayer — engine stores internally
    engine->createControlLayer<DummyPlanner>("my_layer", ContinuousPolicy{10.0});

    auto map_server    = engine->getMapServer<DummyMap>("my_map");
    auto control_layer = engine->getControlLayer<DummyPlanner>("my_layer");

    // 3. Bind map to layer
    control_layer->bind_map_view(map_server, [node](const DummyMap& map_data, const std::string& active_planner) {
            RCLCPP_INFO(node->get_logger(), "Binding map to planner: %s", active_planner.c_str());
            return std::make_shared<ASimpleMapView>(map_data);
    });

    // 5. Post-plan: publish cmd_vel // TODO： 不止支持cmd_vel
    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);
    control_layer->set_on_post_plan(
        [cmd_vel_pub](finenav::core::PostPlanContext& ctx) -> bool {
            if (ctx.is_success && !ctx.result_traj.twists.empty()) {
                cmd_vel_pub->publish(ctx.result_traj.twists[0]);
            }
            return ctx.is_success;
        });

    // 6. Connect pipeline
    engine->connectPipeline({"line_path", "plan_result"}, control_layer);


    // 7. Navigate — C++ API, no ROS action client needed
    geometry_msgs::msg::Pose goal;
    goal.position.x = 5.0;
    goal.position.y = 3.0;
    engine->navigateTo(goal);

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine ready.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
