// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Getting started: assemble a complete navigation stack using FineNavEngine.

#include <memory>
#include <vector>

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

    // 1. Load the algorithm .so's.  They are NOT linked into this executable
    //    (see getting_started/CMakeLists.txt): a self-registering .so that the
    //    executable references no symbol from would be dropped by --as-needed and
    //    its static-init registrar would never run.  dlopen forces the registrar
    //    in each .so to fire, which populates the shared AlgoRegistry.
    //    "finenav_example/lib" resolves to <prefix>/lib where the two libraries
    //    were installed; the scan loads every *.so it finds.
    engine->loadAlgoPlugins({"finenav_example/lib"});

    // 2. Create MapServer by algorithm name.
    //    DummyMap self-registered as "dummy_map" (FINENAV_REGISTER_MAP in its .cpp);
    //    the factory runs inside libdummy_map.so, so the engine never sees the type.
    engine->createMapServerByName("dummy_map", "my_map");

    // 3. Create ControlLayer by algorithm name.
    //    libdummy_planner.so self-registered as "dummy_planner".
    engine->createLayerByName("dummy_planner", "my_layer", core::ContinuousPolicy{10.0});

    // 4. Bind map to layer.
    //    bind_map_view is a typed ControlLayer<DummyPlanner> method — recover the
    //    concrete types (dynamic_pointer_cast only, no ctor instantiation here;
    //    RTLD_GLOBAL keeps the RTTI unified across the DSO boundary).
    auto typed_map   = engine->getMapServer<DummyMap>("my_map");
    auto typed_layer = engine->getControlLayer<DummyPlanner>("my_layer");
    typed_layer->bind_map_view(typed_map, [](const DummyMap& map_data) {
            return std::make_shared<ASimpleMapView>(map_data);
    });

    // 5. Wire tree
    engine->wireControlPipeline({
        _root >> typed_layer
    });

    RCLCPP_INFO(node->get_logger(), "FineNav-Engine ready.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
