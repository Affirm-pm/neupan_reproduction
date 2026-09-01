// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_engine/finenav_engine.hpp"

#include <chrono>
#include <filesystem>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <behaviortree_cpp/utils/shared_library.h>

#include "finenav_core/common/plan_registry.hpp"
#include "finenav_core/common/map_registry.hpp"
#include "finenav_core/common/mapview_registry.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav {

// ══════════════════════════════════════════════════════════════════════════════
//  Constructor
// ══════════════════════════════════════════════════════════════════════════════

FineNavEngine::FineNavEngine(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node) {

    localizer_ = std::make_unique<finenav::localizer::FineNavLocalizer>(node_);
    navigator_ = std::make_unique<finenav::navigator::BtNavigator>(node_, localizer_.get());
    nav_client_ = rclcpp_action::create_client<NavAction>(node_, navigator_->getActionName());

    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped& msg) { navigateTo(msg.pose); });
}

// ══════════════════════════════════════════════════════════════════════════════
//  MapServer removal
// ══════════════════════════════════════════════════════════════════════════════

bool FineNavEngine::removeMapServer(const std::string& name) {
    return maps_.erase(name) > 0;
}

// ══════════════════════════════════════════════════════════════════════════════
//  ControlLayer removal
// ══════════════════════════════════════════════════════════════════════════════

bool FineNavEngine::removeControlLayer(const std::string& name) {
    return layers_.erase(name) > 0;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Factory by name (runtime reflection via PlanRegistry / MapRegistry)
// ══════════════════════════════════════════════════════════════════════════════

std::shared_ptr<finenav::core::IControlLayer> FineNavEngine::createLayerByName(
    const std::string& algo_name, const std::string& layer_name,
    finenav::core::ExecutionPolicyVariant policy)
{
    auto layer = finenav::core::PlanRegistry::instance().createLayer(
        node_, algo_name, layer_name, std::move(policy), localizer_.get());
    if (!layer) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createLayerByName('{}', '{}'): planner not registered. "
            "Call loadAlgoPlugins() first if the algorithm lives in a plugin .so.",
            algo_name, layer_name);
        return nullptr;
    }
    navigator_->master().manageLayer(layer_name);
    layers_[layer_name] = layer;
    FINENAV_LOG_INFO(node_->get_logger(),
        "createLayerByName('{}', '{}'): layer created.", algo_name, layer_name);
    return layer;
}

std::shared_ptr<finenav::core::IMapResource> FineNavEngine::createMapServerByName(
    const std::string& algo_name, const std::string& server_name)
{
    auto server = finenav::core::MapRegistry::instance().createMap(
        node_, algo_name, server_name, localizer_.get());
    if (!server) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createMapServerByName('{}', '{}'): map not registered. "
            "Call loadAlgoPlugins() first if the algorithm lives in a plugin .so.",
            algo_name, server_name);
        return nullptr;
    }
    maps_[server_name] = server;
    FINENAV_LOG_INFO(node_->get_logger(),
        "createMapServerByName('{}', '{}'): map server created.", algo_name, server_name);
    return server;
}

// ══════════════════════════════════════════════════════════════════════════════
//  MapView assembly (runtime reflection via MapViewRegistry + PlanRegistry + MapRegistry)
// ══════════════════════════════════════════════════════════════════════════════

std::shared_ptr<finenav::core::IMapResource> FineNavEngine::ensureMapServer(
    const std::string& algo_name)
{
    auto it = maps_.find(algo_name);
    if (it != maps_.end()) return it->second;
    return createMapServerByName(algo_name, algo_name);
}

std::shared_ptr<finenav::core::IControlLayer> FineNavEngine::createLayerByMapView(
    const std::string& mapview_name, const std::string& layer_name,
    finenav::core::ExecutionPolicyVariant policy)
{
    auto entry = finenav::core::MapViewRegistry::instance().lookup(mapview_name);
    if (!entry) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createLayerByMapView: MapView '{}' is not registered. "
            "Call loadAlgoPlugins() first if it lives in a plugin .so.",
            mapview_name);
        return nullptr;
    }

    auto& plans = finenav::core::PlanRegistry::instance();
    auto& maps  = finenav::core::MapRegistry::instance();

    // Resolve the planner type the view bridges → its registered algo name.
    auto planner_name = plans.findPlannerNameByType(entry->planner_type);
    if (!planner_name) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createLayerByMapView('{}'): planner type '{}' is not registered as an algo.",
            mapview_name, entry->planner_type);
        return nullptr;
    }

    // Resolve + dedup the map server(s) the view depends on.
    std::vector<std::shared_ptr<finenav::core::IMapResource>> map_resources;
    map_resources.reserve(entry->map_types.size());
    for (const auto& map_type : entry->map_types) {
        auto map_name = maps.findMapNameByType(map_type);
        if (!map_name) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "createLayerByMapView('{}'): map type '{}' is not registered as an algo.",
                mapview_name, map_type);
            return nullptr;
        }
        map_resources.push_back(ensureMapServer(*map_name));
    }

    // Build the view: hand build() a MapAccess carrying the resolved maps.
    std::vector<finenav::core::MapHandle> handles;
    handles.reserve(map_resources.size());
    for (const auto& res : map_resources) {
        handles.push_back({res->mapTypeId(), res->mapData()});
    }
    finenav::core::MapAccess access(std::move(handles));
    auto view = entry->build(access);
    if (!view) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createLayerByMapView('{}'): build() returned null.", mapview_name);
        return nullptr;
    }

    // Create the layer for the resolved planner and attach the built view.
    auto layer = createLayerByName(*planner_name, layer_name, std::move(policy));
    if (!layer) return nullptr;
    layer->attachMapView(std::move(view), std::move(map_resources));
    FINENAV_LOG_INFO(node_->get_logger(),
        "createLayerByMapView('{}', '{}'): assembled layer over planner '{}'.",
        mapview_name, layer_name, *planner_name);
    return layer;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Plugin discovery (dlopen)
// ══════════════════════════════════════════════════════════════════════════════

bool FineNavEngine::loadAlgoPlugins(const std::vector<std::string>& plugins)
{
    int loaded_total = 0;
    for (const auto& plugin_param : plugins) {
        const auto slash = plugin_param.find('/');
        if (slash == std::string::npos) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "loadAlgoPlugins: '{}' is not in 'pkg_name/sub_folder' form.", plugin_param);
            continue;
        }
        const auto pkg_name = plugin_param.substr(0, slash);
        const auto sub_folder = plugin_param.substr(slash + 1);

        std::filesystem::path plugin_directory;
        try {
            plugin_directory = std::filesystem::path(
                ament_index_cpp::get_package_prefix(pkg_name)) / sub_folder;
        }
        catch (const ament_index_cpp::PackageNotFoundError&) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "loadAlgoPlugins: package '{}' not found.", pkg_name);
            continue;
        }
        if (!std::filesystem::exists(plugin_directory)) {
            FINENAV_LOG_WARN(node_->get_logger(),
                "loadAlgoPlugins: directory not found: {}", plugin_directory.string());
            continue;
        }

        int loaded_count = 0;
        for (const auto& entry :
             std::filesystem::recursive_directory_iterator(plugin_directory)) {
            if (entry.path().extension() != ".so") continue;
            try {
                // dlopen alone is enough — static-init runs the self-registrars.
                // RTLD_GLOBAL (BT::SharedLibrary's default) keeps cross-DSO RTTI working.
                plugin_handles_.push_back(
                    std::make_shared<BT::SharedLibrary>(entry.path().string()));
                ++loaded_count;
            } catch (const std::exception& e) {
                FINENAV_LOG_ERROR(node_->get_logger(),
                    "loadAlgoPlugins: failed to dlopen '{}': {}", entry.path().filename().string(), e.what());
            }
        }
        FINENAV_LOG_INFO(node_->get_logger(),
            "loadAlgoPlugins: scanned '{}' — loaded {} plugin .so(s).",
            plugin_directory.string(), loaded_count);
        loaded_total += loaded_count;
    }
    return loaded_total > 0;
}

// ══════════════════════════════════════════════════════════════════════════════
//  connectEdge (private)
// ══════════════════════════════════════════════════════════════════════════════

void FineNavEngine::connectEdge(const std::string& from,
                                std::shared_ptr<finenav::core::IControlLayer> to) {
    // Root connection (_root >> layer): no upstream node exists, nothing to
    // subscribe to.  Subscribing results in a dead /__feedback topic.
    if (!from.empty()) {
        to->connectFrom(finenav::util::TaskChannel::feedbackTopic(from));
    }
    navigator_->pipe().addEdge(from, to->layerName());
}

void FineNavEngine::connectEdge(const std::string& from, const std::string& to) {
    auto it = layers_.find(to);
    if (it == layers_.end()) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "wire: layer '{}' not found.", to);
        return;
    }
    connectEdge(from, it->second);
}

// ══════════════════════════════════════════════════════════════════════════════
//  wire (public, fluent)
// ══════════════════════════════════════════════════════════════════════════════

void FineNavEngine::wireControlPipeline(std::initializer_list<ControlPipeEdge> edges) {
    for (auto& edge : edges) {
        for (size_t i = 1; i < edge.chain.size(); ++i) {
            connectEdge(edge.chain[i - 1], edge.chain[i]);
        }
    }
}

// ══════════════════════════════════════════════════════════════════════════════
//  Navigation control
// ══════════════════════════════════════════════════════════════════════════════

void FineNavEngine::navigateTo(const geometry_msgs::msg::Pose& goal,
                               const std::string& bt_id) {
    const std::string tree_id = bt_id.empty() ? default_bt_id_ : bt_id;
    if (tree_id.empty()) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "navigateTo: behavior tree ID is empty. "
            "Pass a bt_id or call setDefaultBehaviorTree() first.");
        return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "Navigator action server unavailable.");
        return;
    }

    auto goal_msg = NavAction::Goal();
    goal_msg.pose.pose         = goal;
    goal_msg.pose.header.stamp = node_->now();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.behavior_tree     = tree_id;

    nav_client_->async_send_goal(goal_msg);
    FINENAV_LOG_INFO(node_->get_logger(),
        "Navigation goal sent: ({:.2f}, {:.2f}, {:.2f})",
        goal.position.x, goal.position.y, goal.position.z);
}

void FineNavEngine::cancelNavigation() {
    nav_client_->async_cancel_all_goals();
    FINENAV_LOG_INFO(node_->get_logger(), "Navigation cancelled.");
}

}  // namespace finenav
