// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <behaviortree_cpp/utils/shared_library.h>

#include "finenav_interfaces/action/navigate_to_pose.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/control_layer.hpp"
#include "finenav_core/plan/i_pipeline_node.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_navigator/chassis_command/i_chassis_command.hpp"
#include "finenav_navigator/navigator.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav {

class FineNavEngine {
public:

    explicit FineNavEngine(const std::shared_ptr<rclcpp::Node>& node);

    FineNavEngine(const FineNavEngine&) = delete;
    FineNavEngine& operator=(const FineNavEngine&) = delete;
    FineNavEngine(FineNavEngine&&) = default;
    FineNavEngine& operator=(FineNavEngine&&) = default;
    ~FineNavEngine() = default;

    void setChassisCommand(std::shared_ptr<navigator::IChassisCommand> cmd) {
        navigator_->setChassisCommand(std::move(cmd));
    }

    /**
     * @brief Set the default behavior tree ID used when navigateTo() is called without one
     */
    void setDefaultBehaviorTree(const std::string& bt_id) { default_bt_id_ = bt_id; }
    [[nodiscard]] const std::string& getDefaultBehaviorTree() const { return default_bt_id_; }

    /**
     * @brief Operation with MapServer
     */
    template <core::Map MapT>
    std::shared_ptr<core::MapServer<MapT>> createMapServer(
        const std::string& algo_name, const std::string& server_name); // TODO: server_name在前

    template <core::Map MapT>
    [[nodiscard]] std::shared_ptr<core::MapServer<MapT>> getMapServer(const std::string& name);

    bool removeMapServer(const std::string& name);

    /**
     * @brief Operation with ControlLayer
     */
    template <core::Planner PlannerT, typename PolicyT = core::SingleShotPolicy>
    std::shared_ptr<core::ControlLayer<PlannerT>> // TODO： 统一命名，CreateControlLayer
    createLayer(const std::string& algo_name, const std::string& name, PolicyT policy = {});

    template <core::Planner PlannerT>
    [[nodiscard]] std::shared_ptr<core::ControlLayer<PlannerT>>
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
    std::shared_ptr<finenav::core::IPipelineNode> createLayerByName( // TODO: CreateControl
        const std::string& algo_name, const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy);

    /// Create a MapServer for a map registered under @p algo_name.
    std::shared_ptr<finenav::core::IMapResource> createMapServerByName(
        const std::string& algo_name, const std::string& server_name);

    /// Assemble a complete ControlLayer from a registered MapView bridge by name.
    std::shared_ptr<finenav::core::IPipelineNode> createLayerByMapView( // TODO: 改个api名字
        const std::string& mapview_name,
        const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy = finenav::core::SingleShotPolicy{});

    /// dlopen every .so under "pkg_name/sub_folder" so its self-registering
    /// algorithms become discoverable by name WITHOUT relinking.  Each entry is
    /// "package/sub_folder" (e.g. {"finenav_example/lib"}).
    bool loadAlgoPlugins(const std::vector<std::string>& plugins); // TODO： 确定下用法，看看能不能只要pkg_name，不要sub_folder

    // ── Connector ─────────────────────────────────────────────────────────

    /// Programmatically declare one named branch of the control pipeline — the
    /// only programmatic entry point for wiring a branch (the JSON path is
    /// loadControlPipeline()).  @p stages is an ordered sequence of layer
    /// names that MUST already have been created (via a layer factory or
    /// loadControlPipeline()'s layer pass).  It does NOT include "_root": the
    /// first stage implicitly consumes _root (navigation-goal injection), and
    /// each subsequent stage consumes its predecessor (fan-out, idempotent).
    /// The branch is then registered under @p branch_name in the navigator's
    /// BranchTable, where the behavior tree activates it with
    /// <ExecuteControlPipelineBranch branch="@p branch_name"/>.
    /// On failure (unknown stage, or addBranch rejecting the branch — e.g.
    /// duplicate branch name or '_root' inside @p stages) an ERROR is logged.
    void wireBranch(const std::string& branch_name, const std::vector<std::string>& stages); // TODO：命名问题，比如 registerControlLayer

    /// Load the control pipeline from a JSON config file:
    /// @code
    /// {
    ///   "control_pipeline": {
    ///     "branches": {
    ///       "direct":  ["route_planner", "mppi"],
    ///       "detour":  ["route_planner", "recovery", "mppi"]
    ///     },
    ///     "layers": {
    ///       "route_planner": { "algo": "astar_planner", "policy": "single_shot" },
    ///       "mppi":          { "mapview": "dummy_mapview", "policy": { "continuous": 10.0 } }
    ///     }
    ///   }
    /// }
    /// @endcode
    /// Same-named layers are created exactly once (dedup by name); each branch
    /// is fan-out wired and registered in the navigator's BranchTable.  Returns
    /// false on parse / validation error.
    bool loadControlPipeline(const std::string& config_path);

    // ── Navigation control ────────────────────────────────────────────────

    void navigateTo(const std::vector<geometry_msgs::msg::Pose>& goals,
                    const std::string& bt_id = "");
    void cancelNavigation();

private:
    /// Return the MapServer already created under @p algo_name, or create one
    /// (server name == algo name) on first use — so multiple MapViews sharing a
    /// map type reuse one server.
    std::shared_ptr<finenav::core::IMapResource> ensureMapServer(const std::string& algo_name); // TODO: 这是干嘛的

    using NavAction = finenav_interfaces::action::NavigateToPose;

    std::shared_ptr<rclcpp::Node> node_;

    std::unordered_map<std::string, std::shared_ptr<finenav::core::IMapResource>> maps_;
    std::unordered_map<std::string, std::shared_ptr<finenav::core::IPipelineNode>> layers_;

    std::unique_ptr<finenav::localizer::FineNavLocalizer> localizer_;
    std::unique_ptr<finenav::navigator::BtNavigator>      navigator_;

    /// dlopen handles for plugin libraries loaded via loadAlgoPlugins().
    std::vector<std::shared_ptr<BT::SharedLibrary>> plugin_handles_;

    rclcpp_action::Client<NavAction>::SharedPtr                     nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    std::string                                                      default_bt_id_;
};

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
