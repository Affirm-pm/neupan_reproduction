// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <initializer_list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "finenav_interfaces/action/navigate_to_pose.hpp"

#include <behaviortree_cpp/utils/shared_library.h>

#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/control_layer.hpp"
#include "finenav_core/plan/i_control_layer.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_navigator/chassis_command/i_chassis_command.hpp"
#include "finenav_navigator/navigator.hpp"
#include "finenav_engine/detail/pipe_syntax.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav {

class FineNavEngine {
public:
    // ── Lifecycle ──────────────────────────────────────────────────────────

    explicit FineNavEngine(const std::shared_ptr<rclcpp::Node>& node);

    FineNavEngine(const FineNavEngine&) = delete;
    FineNavEngine& operator=(const FineNavEngine&) = delete;
    FineNavEngine(FineNavEngine&&) = default;
    FineNavEngine& operator=(FineNavEngine&&) = default;
    ~FineNavEngine() = default;

    // ── Accessors ──────────────────────────────────────────────────────────

    const std::shared_ptr<rclcpp::Node>& getNode() const { return node_; }

    void setChassisCommand(std::shared_ptr<finenav::navigator::IChassisCommand> cmd) {
        navigator_->setChassisCommand(std::move(cmd));
    }

    /// Set the default behavior tree ID used when navigateTo() is called without one
    /// (e.g. from the /goal_pose topic subscriber).
    void setDefaultBehaviorTree(const std::string& bt_id) { default_bt_id_ = bt_id; }
    [[nodiscard]] const std::string& getDefaultBehaviorTree() const { return default_bt_id_; }

    // ── Factory: map ──────────────────────────────────────────────────────

    template <finenav::core::Map MapT>
    std::shared_ptr<finenav::core::MapServer<MapT>> createMapServer(
        const std::string& algo_name, const std::string& server_name);

    template <finenav::core::Map MapT>
    [[nodiscard]] std::shared_ptr<finenav::core::MapServer<MapT>> getMapServer(const std::string& name);

    bool removeMapServer(const std::string& name);

    // ── Factory: layer ────────────────────────────────────────────────────

    template <finenav::core::Planner PlannerT, typename PolicyT = finenav::core::SingleShotPolicy>
    std::shared_ptr<finenav::core::ControlLayer<PlannerT>>
    createLayer(const std::string& algo_name, const std::string& name, PolicyT policy = {});

    template <finenav::core::Planner PlannerT>
    [[nodiscard]] std::shared_ptr<finenav::core::ControlLayer<PlannerT>>
    getControlLayer(const std::string& name);

    bool removeControlLayer(const std::string& name);

    // ── Factory by name (runtime reflection via PlanRegistry / MapRegistry) ────
    // The algorithm .so self-registers its factory under the registration name
    // (#PARAM_NS from FINENAV_REGISTER_PLANNER / FINENAV_REGISTER_MAP) at
    // static-init (dlopen or link).  These look the factory up by name and build
    // the layer / map server inside the plugin, so the engine never needs the
    // concrete type.  Unknown names fail loudly (nullptr + ERROR) — no silent
    // fallback.  The concrete type is recovered later with getControlLayer<T>() /
    // getMapServer<T>() (for typed operations).

    /// Create a ControlLayer for a planner registered under @p algo_name.
    std::shared_ptr<finenav::core::IControlLayer> createLayerByName(
        const std::string& algo_name, const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy);

    /// Create a MapServer for a map registered under @p algo_name.
    std::shared_ptr<finenav::core::IMapResource> createMapServerByName(
        const std::string& algo_name, const std::string& server_name);

    /// Assemble a complete ControlLayer from a registered MapView bridge by name.
    /// The MapView registry entry declares only TYPES (planner + maps); this
    /// resolves those types back to registered algo names, (re)uses dedup'd
    /// MapServer(s), builds the IMapView and attaches it to a freshly created
    /// layer.  The end user names ONLY the MapView — never a planner/map name.
    std::shared_ptr<finenav::core::IControlLayer> createLayerByMapView(
        const std::string& mapview_name,
        const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy = finenav::core::SingleShotPolicy{});

    /// dlopen every .so under "pkg_name/sub_folder" so its self-registering
    /// algorithms become discoverable by name WITHOUT relinking.  Each entry is
    /// "package/sub_folder" (e.g. {"finenav_example/lib"}).  The handles are kept
    /// alive for the engine's lifetime (BT::SharedLibrary itself never unloads).
    /// Must be called before the first by-name create.
    bool loadAlgoPlugins(const std::vector<std::string>& plugins);

    // ── Connector ─────────────────────────────────────────────────────────

    /// Wire multiple paths into a tree.  Each path is a chain built with >>.
    /// @code
    ///   engine->wire({
    ///       _root >> rte >> det >> mppi,
    ///              rte >> rot
    ///   });
    /// @endcode
    void wireControlPipeline(std::initializer_list<ControlPipeEdge> edges);

    // ── Navigation control ────────────────────────────────────────────────

    void navigateTo(const geometry_msgs::msg::Pose& goal,
                    const std::string& bt_id = "");
    void cancelNavigation();

private:
    void connectEdge(const std::string& from, const std::string& to);
    void connectEdge(const std::string& from,
                     std::shared_ptr<finenav::core::IControlLayer> to);

    /// Return the MapServer already created under @p algo_name, or create one
    /// (server name == algo name) on first use — so multiple MapViews sharing a
    /// map type reuse one server.
    std::shared_ptr<finenav::core::IMapResource> ensureMapServer(const std::string& algo_name);

    using NavAction = finenav_interfaces::action::NavigateToPose;

    std::shared_ptr<rclcpp::Node> node_;

    std::unordered_map<std::string, std::shared_ptr<finenav::core::IMapResource>> maps_;
    std::unordered_map<std::string, std::shared_ptr<finenav::core::IControlLayer>> layers_;

    std::unique_ptr<finenav::localizer::FineNavLocalizer> localizer_;
    std::unique_ptr<finenav::navigator::BtNavigator>      navigator_;

    /// dlopen handles for plugin libraries loaded via loadAlgoPlugins().
    std::vector<std::shared_ptr<BT::SharedLibrary>> plugin_handles_;

    rclcpp_action::Client<NavAction>::SharedPtr                     nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    std::string                                                      default_bt_id_;
};

// ══════════════════════════════════════════════════════════════════════════════
//  Factory: map
// ══════════════════════════════════════════════════════════════════════════════

template <finenav::core::Map MapT>
std::shared_ptr<finenav::core::MapServer<MapT>>
FineNavEngine::createMapServer(
    const std::string& algo_name, const std::string& server_name) {
    auto server = std::make_shared<finenav::core::MapServer<MapT>>(
        node_, algo_name, server_name, localizer_.get());
    maps_[server_name] = server;
    return server;
}

template <finenav::core::Map MapT>
std::shared_ptr<finenav::core::MapServer<MapT>> FineNavEngine::getMapServer(const std::string& name) {
    auto it = maps_.find(name);
    if (it == maps_.end()) return nullptr;
    auto casted = std::dynamic_pointer_cast<finenav::core::MapServer<MapT>>(it->second);
    if (!casted) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "getMapServer('{}'): type mismatch.", name);
        return nullptr;
    }
    return casted;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Factory: layer
// ══════════════════════════════════════════════════════════════════════════════

template <finenav::core::Planner PlannerT, typename PolicyT>
std::shared_ptr<finenav::core::ControlLayer<PlannerT>>
FineNavEngine::createLayer(
    const std::string& algo_name, const std::string& name, PolicyT policy) {
    navigator_->master().manageLayer(name);
    auto layer = std::make_shared<finenav::core::ControlLayer<PlannerT>>(
        node_, algo_name, name, std::move(policy), localizer_.get());
    layers_[name] = layer;
    return layer;
}

template <finenav::core::Planner PlannerT>
std::shared_ptr<finenav::core::ControlLayer<PlannerT>>
FineNavEngine::getControlLayer(const std::string& name) {
    auto it = layers_.find(name);
    if (it == layers_.end()) return nullptr;
    return std::dynamic_pointer_cast<finenav::core::ControlLayer<PlannerT>>(it->second);
}

}  // namespace finenav
