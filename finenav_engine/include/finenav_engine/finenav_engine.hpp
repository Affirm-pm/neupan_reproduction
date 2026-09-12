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
#include "finenav_core/map/i_map_resource.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/i_pipeline_node.hpp"
#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_navigator/chassis_command/i_chassis_command.hpp"
#include "finenav_navigator/navigator.hpp"

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
     * @brief Read-only access to the current robot state.
     *
     * Returns the narrow read-only query interface of the localizer (the
     * RobotStateProvider base); `pushState()` is protected, so callers can only
     * query — `getRobotState()` / `getRobotState(t)`.  The engine owns the
     * provider, so the returned pointer is valid for the engine's lifetime.
     */
    [[nodiscard]] const core::RobotStateProvider* robotStateProvider() const {
        return localizer_.get();
    }

    // ── Factory by name (runtime reflection via PlanRegistry / MapRegistry) ────
    // The algorithm .so self-registers its factory under the registration name
    // (#PARAM_NS from FINENAV_REGISTER_PLANNER / FINENAV_REGISTER_MAP) at
    // static-init (dlopen or link).  These look the factory up by name and build
    // the layer / map server inside the plugin, so the engine never needs the
    // concrete type.  Unknown names fail loudly (nullptr + ERROR) — no silent
    // fallback.
    //
    // There is deliberately no by-TYPE counterpart (`createLayer<T>()`,
    // `getMapServer<T>()`).  Those were removed before 1.0: they forced the
    // downstream user to name a planner/map type that lives inside a plugin
    // DSO — exactly what the by-name factory exists to avoid — and they handed
    // back handles that reached into a layer's or a map's flow.  The downstream
    // user assembles by name and tunes parameters; the flow belongs to the
    // algorithm developer.
    //
    // For the same reason these three report success, not a handle.  A returned
    // IPipelineNode would hand the caller connectFrom() / attachWorldView() /
    // getPrivateNode() — the very flow the by-name factory exists to keep on the
    // algorithm developer's side of the line — and the only thing the caller can
    // legitimately do with it is test whether the assembly worked.  The
    // pointer-returning forms live in make*() below, where the ENGINE still has
    // to drive what it just built.

    /// Assemble the pipeline layer for the planner registered under @p algo_name.
    /// @return false if no planner is registered under that name (ERROR logged).
    [[nodiscard]] bool createLayerByName( // TODO: CreateControl
        const std::string& algo_name, const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy);

    /// Create the map server for the map registered under @p algo_name.
    /// @return false if no map is registered under that name (ERROR logged).
    [[nodiscard]] bool createMapServerByName(
        const std::string& algo_name, const std::string& server_name);

    /// Assemble a complete layer from a registered WorldView bridge by name.
    /// @return false if the WorldView, its planner or one of its maps is not
    ///         registered, or if building the view failed (ERROR logged).
    [[nodiscard]] bool createLayerByWorldView( // TODO: 改个api名字
        const std::string& world_view_name,
        const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy = finenav::core::EventDrivenPolicy{});

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
    ///       "route_planner": { "algo": "astar_planner", "policy": "event_driven" },
    ///       "mppi":          { "world_view": "dummy_world_view", "policy": { "periodic": 10.0 } }
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
    // ── Pointer-returning factory forms ──────────────────────────────────────
    // Everything the public bool API does, but handing back the object.  Private
    // because holding one of these IS the capability the public form withholds:
    // assembling a WorldView needs the layer's private sub-node and its
    // attachWorldView(); resolving a cached map needs its resource handle.

    std::shared_ptr<finenav::core::IPipelineNode> makeLayerByName(
        const std::string& algo_name, const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy);

    std::shared_ptr<finenav::core::IMapResource> makeMapServerByName(
        const std::string& algo_name, const std::string& server_name);

    /// Return the MapServer already created under @p algo_name, or create one
    /// (server name == algo name) on first use — so multiple WorldViews sharing a
    /// map type reuse one server.
    std::shared_ptr<finenav::core::IMapResource> ensureMapServer(const std::string& algo_name); // TODO: 这是干嘛的

    /// Drop a half-built layer from the engine's bookkeeping.  Private: the
    /// downstream user does not get to dismantle pipeline stages — which layer
    /// exists is the algorithm developer's decision, made at assembly time.
    bool removeControlLayer(const std::string& name);

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

}  // namespace finenav
