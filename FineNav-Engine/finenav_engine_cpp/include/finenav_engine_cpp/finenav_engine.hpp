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
#include "finenav_common/frames.hpp"
#include "finenav_core/map/i_map_resource.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/plan/i_pipeline_node.hpp"

namespace finenav {

namespace localizer { class FineNavLocalizer; }
namespace navigator {
class BtNavigator;
class IChassisCommand;
}  // namespace navigator

/**
 * @brief Top-level orchestrator: localizer + navigator + the by-name control pipeline.
 *
 * Layers and map servers are assembled by registration name (runtime reflection), so the
 * engine never names a concrete planner/map/world-view type; an unknown name fails loudly
 * (ERROR logged, no silent fallback).
 *
 * The factories report success with `bool`, not a handle — a returned handle would hand the
 * caller connectFrom()/attachWorldView(), i.e. the flow that belongs to the algorithm
 * developer, not the main.cpp user.
 */
class FineNavEngine {
public:

    /**
     * @brief Construct the engine around an existing ROS node.
     * @param node  Host node
     */
    explicit FineNavEngine(std::shared_ptr<rclcpp::Node> node);

    FineNavEngine(const FineNavEngine&) = delete;
    FineNavEngine& operator=(const FineNavEngine&) = delete;
    FineNavEngine(FineNavEngine&&);
    FineNavEngine& operator=(FineNavEngine&&);
    ~FineNavEngine();

    /**
     * @brief Set the world and robot frame names written into outgoing message headers.
     * Example usage:
     * @code
     *   engine.setFrameNames({.world_fixed = "map", .robot = "base_footprint"});
     * @endcode
     * @param names  World-fixed and robot frame names.
     * @return false if a layer/map already exists or the names were already set.
     * @note Call this before any layer or map is assembled. Setting frame names is
     *       one-shot: a name changed after messages were already published would
     *       silently invalidate those messages, so a second call is ignored.
     */
    bool setFrameNames(common::FrameNames names);

    /**
     * @brief Set the command sink that publishes trajectory as a chassis message.
     * If never set, the navigator defaults to a ChassisCmdVel publishing on /cmd_vel, geometry_msgs::msg::Twist
     * Example usage:
     * @code
     *   engine.setChassisCommand(
     *       std::make_shared<finenav::navigator::ChassisCmdVel>(node, "/cmd_vel"));
     * @endcode
     * @param cmd  The chassis command.
     */
    void setChassisCommand(std::shared_ptr<navigator::IChassisCommand> cmd);

    /**
     * @brief Set the behavior tree id used by navigateTo() when none is passed.
     * @param bt_id  Behavior tree id, as registered with the BT engine.
     * @note An empty id means "no default": navigateTo() then errors unless an explicit bt_id is passed.
     */
    void setDefaultBehaviorTree(const std::string& bt_id);

    /**
     * @brief Navigate to a series of goal.
     *
     * @param goals  World-frame goal poses, in visit order.
     * @param bt_id  Behavior tree id; uses the default (setDefaultBehaviorTree) when empty.
     */
    void navigateTo(const std::vector<geometry_msgs::msg::Pose>& goals, const std::string& bt_id = "");

    /**
     * @brief Cancel the in-flight navigation goal.
     */
    void cancelNavigation();

    /**
     * @brief dlopen the algorithm .so's each named package publishes.
     *
     * Each entry is a bare package name; the .so's to load come from that
     * package's `finenav_algo_plugins` ament-index resource (written at build
     * time by finenav_register_algo_plugin()), whose content is one
     * prefix-relative library path per line.  A .so that loads but registers no
     * algorithm is silently skipped; the call logs a one-line summary of the
     * planners / maps / world views that actually self-registered.
     * Example usage:
     * @code
     *   engine.loadAlgoPlugins({"finenav_example"});
     * @endcode
     * @param plugins  Package names (e.g. {"finenav_example"}).
     * @return true if at least one planner, map, or world view registered.
     */
    bool loadAlgoPlugins(const std::vector<std::string>& plugins);

    /**
     * @brief Create a control-pipeline layer for an algorithm by registration name.
     * Example usage:
     * @code
     *   engine.createControlLayer("mppi_world_view", "mppi");   // map-reading
     *   engine.createControlLayer("my_decorator", "decorate");  // map-less
     * @endcode
     * @param algo_name   WorldView name (map-reading) or planner name (map-less).
     * @param layer_name  Unique layer name; it becomes the stage's topic name.
     * @param policy      Execution policy (event-driven or periodic).
     * @return false if neither a WorldView nor a map-less Planner is registered
     *         under that name.
     */
    bool createControlLayer(
        const std::string& algo_name, const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy = finenav::core::EventDrivenPolicy{});

    /**
     * @brief Declare a named control branch as an ordered list of already-created layers.
     *
     * Each entry in @p stages is a layer name — the name passed to createControlLayer() —
     * which also serves as that layer's stage in the branch, so one name may appear in
     * several branches (fan-out).  The first stage implicitly consumes _root
     * (navigation-goal injection) and each later stage consumes its predecessor, so
     * "_root" must not appear in @p stages.  The branch is registered in the navigator's
     * BranchTable, where a behavior tree activates it with
     * <ExecuteControlBranch branch="@p branch_name"/>.
     *
     * @param branch_name  Unique branch name.
     * @param stages       Ordered layer names, in pipeline order.
     * @return false on failure (empty stages, an unknown stage, or addBranch rejecting it).
     */
    bool registerControlBranch(const std::string& branch_name, const std::vector<std::string>& stages);

    /**
     * @brief Load layers and branches from a JSON config file.
     *
     * @code
     * {
     *   "control_pipeline": {
     *     "branches": { "direct": ["route_planner", "mppi"] },
     *     "layers": {
     *       "route_planner": { "algo": "astar_planner", "policy": "event_driven" },
     *       "mppi":          { "world_view": "dummy_world_view", "policy": { "periodic": 10.0 } }
     *     }
     *   }
     * }
     * @endcode
     * Same-named layers are created exactly once (dedup by name); each branch is fan-out
     * wired and registered in the navigator's BranchTable.
     *
     * @param config_path  Path to the JSON file.
     * @return false on parse / validation error. */
    bool loadControlPipeline(const std::string& config_path); // TODO: 配置文件（schema）创造

private:
    std::shared_ptr<finenav::core::IPipelineNode> makeControlLayerByPlanner(
        const std::string& planner_name, const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy);

    std::shared_ptr<finenav::core::IMapResource> makeMapServer(
        const std::string& map_name, const std::string& server_name);

    bool createControlLayerFromWorldView(
        const std::string& world_view_name, const std::string& layer_name,
        finenav::core::ExecutionPolicyVariant policy);

    /// Get-or-create the map server under @p map_name (server name == map name).
    std::shared_ptr<finenav::core::IMapResource> ensureMapServer(const std::string& map_name);

    /// Drop a half-built layer from bookkeeping.
    bool removeControlLayer(const std::string& name);

    using NavAction = finenav_interfaces::action::NavigateToPose;

    std::shared_ptr<rclcpp::Node> node_;
    std::unordered_map<std::string, std::shared_ptr<finenav::core::IMapResource>> maps_;
    std::unordered_map<std::string, std::shared_ptr<finenav::core::IPipelineNode>> layers_;
    std::unique_ptr<finenav::localizer::FineNavLocalizer> localizer_;
    std::unique_ptr<finenav::navigator::BtNavigator> navigator_;
    std::vector<std::shared_ptr<BT::SharedLibrary>> plugin_handles_;
    rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    std::string default_bt_id_;
};

}  // namespace finenav
