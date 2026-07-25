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

using namespace finenav;

using example::DummyPlanner;
using example::DummyMap;

/**
 *  @brief Define a MapView, which determine the rule for planner
 *  @details In this example, the rule is quite simple: the planner only considers the map occupied if the position is occupied in the map.
 */
class ASimpleMapView : public DummyPlanner::IMapView {
public:
    ASimpleMapView(const DummyMap& map_ptr) : map_(map_ptr) {}
    bool isWalkable(const core::Position3D&) const override {
        return !map_.isOccupied(core::Position3D());
    }
private:
    const DummyMap& map_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("hello_finenav");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    // 0. Set the default behavior tree (used by /goal_pose and navigateTo() without bt_id).
    engine->setDefaultBehaviorTree("GettingStartedNavigate");

    // 1. Create MapServer
    auto map_server = engine->createMapServer<DummyMap>("my_map");

    // 2. Create ControlLayer
    auto layer = engine->createLayer<DummyPlanner>("my_layer", core::ContinuousPolicy{10.0});

    // 3. Bind map to layer
    layer->bind_map_view(map_server, [](const DummyMap& map_data, const std::string&) {
            return std::make_shared<ASimpleMapView>(map_data);
    });

    // 4. Wire tree
    engine->wireControlPipeline({
        _root >> layer
    });

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine ready.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
