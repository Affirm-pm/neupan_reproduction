// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_engine_cpp/finenav_engine.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>

#include <ament_index_cpp/get_resource.hpp>
#include <behaviortree_cpp/utils/shared_library.h>
#include <nlohmann/json.hpp>

#include "finenav_core/plan/detail/plan_registry.hpp"
#include "finenav_core/map/detail/map_registry.hpp"
#include "finenav_core/worldview/detail/world_view_registry.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_navigator/navigator.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav {

namespace {

/// Parse a JSON `policy` value into the framework's ExecutionPolicyVariant.
/// Accepted forms (see the design spec §1.1):
///   "event_driven" | "periodic"
///   <number>                                     → PeriodicPolicy{number}
///   {"periodic": hz} | {"frequency_hz": hz}
///   {"type": "periodic"|"event_driven", "frequency_hz": hz}
/// Missing / null → EventDrivenPolicy (general server default).
/// Returns std::nullopt on any unrecognized policy value.
std::optional<finenav::core::ExecutionPolicyVariant> parsePolicy(const nlohmann::json& policy) {
    using finenav::core::PeriodicPolicy;
    using finenav::core::EventDrivenPolicy;

    if (policy.is_null()) return EventDrivenPolicy{};

    if (policy.is_string()) {
        const std::string s = policy.get<std::string>();
        if (s == "event_driven") return EventDrivenPolicy{};
        if (s == "periodic")     return PeriodicPolicy{};
        return std::nullopt;
    }

    if (policy.is_number()) return PeriodicPolicy{policy.get<double>()};

    if (policy.is_object()) {
        double hz = 10.0;
        bool have_hz = false;
        if (policy.contains("frequency_hz") && policy["frequency_hz"].is_number()) {
            hz = policy["frequency_hz"].get<double>();
            have_hz = true;
        }
        if (policy.contains("periodic") && policy["periodic"].is_number()) {
            hz = policy["periodic"].get<double>();
            have_hz = true;
        }
        if (policy.contains("type") && policy["type"].is_string()) {
            const std::string t = policy["type"].get<std::string>();
            if (t == "periodic")     return PeriodicPolicy{have_hz ? hz : 10.0};
            if (t == "event_driven") return EventDrivenPolicy{};
            return std::nullopt;
        }
        if (have_hz) return PeriodicPolicy{hz};
        return std::nullopt;
    }

    return std::nullopt;
}

}  // namespace

// ══════════════════════════════════════════════════════════════════════════════
//  Constructor
// ══════════════════════════════════════════════════════════════════════════════

FineNavEngine::FineNavEngine(std::shared_ptr<rclcpp::Node> node)
    : node_(std::move(node)) {

    localizer_ = std::make_unique<finenav::localizer::FineNavLocalizer>(node_);
    navigator_ = std::make_unique<finenav::navigator::BtNavigator>(node_, localizer_.get());
    nav_client_ = rclcpp_action::create_client<NavAction>(node_, navigator_->getActionName());

    goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped& msg) { navigateTo({msg.pose}); });
}

FineNavEngine::FineNavEngine(FineNavEngine&&) = default;
FineNavEngine& FineNavEngine::operator=(FineNavEngine&&) = default;
FineNavEngine::~FineNavEngine() = default;

void FineNavEngine::setChassisCommand(std::shared_ptr<navigator::IChassisCommand> cmd) {
    navigator_->setChassisCommand(std::move(cmd));
}

void FineNavEngine::setDefaultBehaviorTree(const std::string& bt_id) {
    default_bt_id_ = bt_id;
}

bool FineNavEngine::setFrameNames(common::FrameNames names) {
    // Ordering guard.  The localizer and navigator built in the constructor read
    // frames() lazily as they publish, so they are unaffected; but a plugin's
    // visualizer takes the frame when the layer is built, so once anything is
    // assembled the value can no longer be honoured everywhere.
    if (!layers_.empty() || !maps_.empty()) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "setFrameNames() ignored — {} layer(s) and {} map(s) already exist. "
            "Frame names must be set before anything is assembled.",
            layers_.size(), maps_.size());
        return false;
    }
    return common::detail::setFrameNames(std::move(names));
}

bool FineNavEngine::removeControlLayer(const std::string& name) {
    const bool was_registered = layers_.erase(name) > 0;
    // createControlLayer() registered this layer's channel with the master
    // (manageLayer), so dropping it from `layers_` alone would leave the master
    // holding a live publisher + subscription pair for a layer that no longer
    // exists — the orphan this call exists to prevent.
    navigator_->master().unmanageLayer(name);
    return was_registered;
}

std::shared_ptr<finenav::core::IPipelineNode> FineNavEngine::makeControlLayerByPlanner(
    const std::string& planner_name, const std::string& layer_name,
    finenav::core::ExecutionPolicyVariant policy)
{
    auto layer = finenav::core::detail::PlanRegistry::instance().createLayer(
        node_, planner_name, layer_name, std::move(policy), localizer_.get());
    if (!layer) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createControlLayer('{}', '{}'): planner not registered. "
            "Call loadAlgoPlugins() first if the algorithm lives in a plugin .so.",
            planner_name, layer_name);
        return nullptr;
    }
    navigator_->master().manageLayer(layer_name);
    layers_[layer_name] = layer;
    FINENAV_LOG_INFO(node_->get_logger(),
        "createControlLayer('{}', '{}'): layer created.", planner_name, layer_name);
    return layer;
}

std::shared_ptr<finenav::core::IMapResource> FineNavEngine::makeMapServer(
    const std::string& map_name, const std::string& server_name)
{
    auto server = finenav::core::detail::MapRegistry::instance().createMap(
        node_, map_name, server_name, localizer_.get());
    if (!server) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "makeMapServer('{}', '{}'): map not registered. "
            "Call loadAlgoPlugins() first if the algorithm lives in a plugin .so.",
            map_name, server_name);
        return nullptr;
    }
    maps_[server_name] = server;
    FINENAV_LOG_INFO(node_->get_logger(),
        "makeMapServer('{}', '{}'): map server created.", map_name, server_name);
    return server;
}

std::shared_ptr<finenav::core::IMapResource> FineNavEngine::ensureMapServer(
    const std::string& map_name)
{
    auto it = maps_.find(map_name);
    if (it != maps_.end()) return it->second;
    return makeMapServer(map_name, map_name);
}

bool FineNavEngine::createControlLayer(
    const std::string& algo_name, const std::string& layer_name,
    finenav::core::ExecutionPolicyVariant policy)
{
    // 1) WorldView name → map-reading planner, fully wired (planner + maps + view).
    if (finenav::core::detail::WorldViewRegistry::instance().lookup(algo_name)) {
        return createControlLayerFromWorldView(algo_name, layer_name, std::move(policy));
    }

    // 2) Planner name → map-less planner.  A map-reading planner reached by its bare
    //    planner name is rejected here (it must go through its WorldView name), so a
    //    missing view can never surface later as "WorldView not attached".
    auto layer = makeControlLayerByPlanner(algo_name, layer_name, std::move(policy));
    if (!layer) return false;   // unknown planner — makeControlLayerByPlanner already logged
    if (layer->requiresWorldView()) {
        removeControlLayer(layer_name);
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createControlLayer('{}', '{}'): planner '{}' reads a map — create it by its "
            "WorldView name, not the bare planner name.",
            algo_name, layer_name, algo_name);
        return false;
    }
    return true;
}

bool FineNavEngine::createControlLayerFromWorldView(
    const std::string& world_view_name, const std::string& layer_name,
    finenav::core::ExecutionPolicyVariant policy)
{
    auto entry = finenav::core::detail::WorldViewRegistry::instance().lookup(world_view_name);
    if (!entry) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createControlLayer: WorldView '{}' is not registered. "
            "Call loadAlgoPlugins() first if it lives in a plugin .so.",
            world_view_name);
        return false;
    }

    auto& plans = finenav::core::detail::PlanRegistry::instance();
    auto& maps  = finenav::core::detail::MapRegistry::instance();

    // Resolve the planner type the view bridges → its registered algo name.
    auto planner_name = plans.findPlannerNameByType(entry->planner_type);
    if (!planner_name) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createControlLayer('{}'): planner type '{}' is not registered as an algo.",
            world_view_name, entry->planner_type);
        return false;
    }

    // Resolve + dedup the map server(s) the view depends on.
    std::vector<std::shared_ptr<finenav::core::IMapResource>> map_resources;
    map_resources.reserve(entry->map_types.size());
    for (const auto& map_type : entry->map_types) {
        auto map_name = maps.findMapNameByType(map_type);
        if (!map_name) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "createControlLayer('{}'): map type '{}' is not registered as an algo.",
                world_view_name, map_type);
            return false;
        }
        map_resources.push_back(ensureMapServer(*map_name));
    }

    // 1) create the layer FIRST — its pnode must exist before build
    auto layer = makeControlLayerByPlanner(*planner_name, layer_name, std::move(policy));
    if (!layer) return false;

    // 2) build MapAccess carrying the resolved maps
    std::vector<finenav::core::MapHandle> handles;
    handles.reserve(map_resources.size());
    for (const auto& res : map_resources) {
        handles.push_back({res->mapTypeId(), res->mapData()});
    }
    finenav::core::MapAccess access(std::move(handles));

    // 3) build the view (its constructor runs here), injecting the layer's pnode
    auto built = entry->build(access, layer->getPrivateNode());
    if (!built.view) {
        // The layer was already registered in layers_/navigator master by
        // makeControlLayerByPlanner(); reclaim it so no orphan layer is left behind.
        removeControlLayer(layer_name);
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createControlLayer('{}'): build() returned no view.", world_view_name);
        return false;
    }

    // 4) attach the view + its param pollable + its prepare() invoker
    layer->attachWorldView(std::move(built), std::move(map_resources));
    FINENAV_LOG_INFO(node_->get_logger(),
        "createControlLayer('{}', '{}'): assembled layer over planner '{}'.",
        world_view_name, layer_name, *planner_name);
    return true;
}

// ament-resource-index plugin discovery.  Each entry is a bare PACKAGE NAME;
// the .so's to dlopen come from that package's `finenav_algo_plugins`
// ament-index resource, whose content is one prefix-relative library path per
// line (written at build time by finenav_register_algo_plugin()).
bool FineNavEngine::loadAlgoPlugins(const std::vector<std::string>& plugins)
{
    constexpr const char* kResourceType = "finenav_algo_plugins";

    for (const auto& pkg_name : plugins) {
        // get_resource() throws std::runtime_error on an EMPTY name (terminates
        // the process), so an empty entry is skipped up front.
        if (pkg_name.empty()) continue;

        std::string content;
        std::string prefix;
        // get_resource() returns false (it does NOT throw) when the package or
        // the resource type is missing — that is the "not a plugin package" case.
        if (!ament_index_cpp::get_resource(kResourceType, pkg_name, content, &prefix)) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "loadAlgoPlugins: package '{}' registers no '{}' resource "
                "(add ament_index_register_resource({} CONTENT ...) to its CMakeLists).",
                pkg_name, kResourceType, kResourceType);
            continue;
        }

        std::istringstream lines(content);
        std::string relative_path;
        while (std::getline(lines, relative_path)) {
            if (relative_path.empty()) continue;
            const auto so_path = std::filesystem::path(prefix) / relative_path;
            if (!std::filesystem::exists(so_path)) {
                FINENAV_LOG_WARN(node_->get_logger(),
                    "loadAlgoPlugins: '{}' lists '{}', which does not exist.",
                    pkg_name, so_path.string());
                continue;
            }
            try {
                // dlopen alone is enough — static-init runs the self-registrars.
                // RTLD_GLOBAL (BT::SharedLibrary's default) keeps cross-DSO RTTI working.
                plugin_handles_.push_back(
                    std::make_shared<BT::SharedLibrary>(so_path.string()));
            } catch (const std::exception& e) {
                FINENAV_LOG_ERROR(node_->get_logger(),
                    "loadAlgoPlugins: failed to dlopen '{}': {}", so_path.filename().string(), e.what());
            }
        }
    }

    // A .so is a plugin only if it self-registered a name; one that dlopens
    // cleanly but registers nothing is silently not a plugin.  This summary is
    // therefore the authority on what actually loaded.
    const auto planners = core::detail::PlanRegistry::instance().names();
    const auto maps     = core::detail::MapRegistry::instance().names();
    const auto views    = core::detail::WorldViewRegistry::instance().names();
    const size_t loaded = planners.size() + maps.size() + views.size();
    FINENAV_LOG_INFO(node_->get_logger(),
        "loadAlgoPlugins: loaded {} plugin(s) — planners=[{}] maps=[{}] world_views=[{}]",
        loaded, common::detail::joinNames(planners), common::detail::joinNames(maps),
        common::detail::joinNames(views));
    return loaded > 0;
}

bool FineNavEngine::registerControlBranch(const std::string& branch_name,
                                          const std::vector<std::string>& stages) {
    if (stages.empty()) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "registerControlBranch('{}'): branch carries no stages.", branch_name);
        return false;
    }
    // Every stage must already exist as a layer (created by a layer factory or
    // by loadControlPipeline()'s layer pass).
    for (const auto& s : stages) {
        if (!layers_.count(s)) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "registerControlBranch('{}'): stage '{}' is not a created layer.", branch_name, s);
            return false;
        }
    }
    // 首 stage 隐式消费 _root（导航目标注入）。connectFrom() 幂等 —— 同一
    // (stage, parent) 对重复出现（跨 branch）只建一个订阅；不同前驱则各自订阅
    // （fan-out）。
    layers_[stages.front()]->connectFrom("_root");
    for (size_t i = 1; i < stages.size(); ++i) {
        layers_[stages[i]]->connectFrom(stages[i - 1]);
    }
    if (!navigator_->pipe().addBranch(branch_name, stages)) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "registerControlBranch('{}'): addBranch rejected (duplicate branch name, empty "
            "stages, duplicate stage, or '_root' inside stages).", branch_name);
        return false;
    }
    return true;
}

bool FineNavEngine::loadControlPipeline(const std::string& config_path) {
    std::ifstream in(config_path);
    if (!in.is_open()) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "loadControlPipeline: cannot open '{}'.", config_path);
        return false;
    }

    nlohmann::json root;
    try {
        in >> root;
    } catch (const std::exception& e) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "loadControlPipeline: failed to parse '{}': {}", config_path, e.what());
        return false;
    }

    // Accept both {"control_pipeline": {...}} and a flat {...}.
    const nlohmann::json* cfg = &root;
    if (root.contains("control_pipeline") && root["control_pipeline"].is_object()) {
        cfg = &root["control_pipeline"];
    }

    bool ok = true;

    // (a) 约化：按 layer 名去重，每个名字只创建一次。
    if (cfg->contains("layers")) {
        const auto& layers = (*cfg)["layers"];
        if (!layers.is_object()) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "loadControlPipeline: 'layers' must be an object.");
            return false;
        }
        for (auto it = layers.begin(); it != layers.end(); ++it) {
            const std::string& layer_name = it.key();
            if (layers_.count(layer_name)) continue;  // dedup: reuse existing layer
            const nlohmann::json& spec = it.value();

            finenav::core::ExecutionPolicyVariant policy = finenav::core::EventDrivenPolicy{};
            if (spec.is_object() && spec.contains("policy")) {
                auto parsed = parsePolicy(spec["policy"]);
                if (!parsed) {
                    FINENAV_LOG_ERROR(node_->get_logger(),
                        "loadControlPipeline: layer '{}' has an unrecognized 'policy' value.", layer_name);
                    ok = false;
                    continue;
                }
                policy = std::move(*parsed);
            }

            std::string algo, world_view;
            if (spec.is_string()) {
                algo = spec.get<std::string>();       // shorthand: algo name
            } else if (spec.is_object()) {
                if (spec.contains("algo") && spec["algo"].is_string()) algo = spec["algo"].get<std::string>();
                if (spec.contains("world_view") && spec["world_view"].is_string()) world_view = spec["world_view"].get<std::string>();
                if (!algo.empty() && !world_view.empty()) {
                    FINENAV_LOG_ERROR(node_->get_logger(),
                        "loadControlPipeline: layer '{}' declares both 'algo' and 'world_view'.", layer_name);
                    ok = false;
                    continue;
                }
            } else {
                FINENAV_LOG_ERROR(node_->get_logger(),
                    "loadControlPipeline: layer '{}' must be a string or an object.", layer_name);
                ok = false;
                continue;
            }

            const std::string& spec_name = !world_view.empty() ? world_view : algo;
            if (!createControlLayer(spec_name, layer_name, std::move(policy))) {
                FINENAV_LOG_ERROR(node_->get_logger(),
                    "loadControlPipeline: failed to create layer '{}'.", layer_name);
                ok = false;
            }
        }
    }

    // (b) branches：wiring fan-out + addBranch。
    if (cfg->contains("branches")) {
        const auto& branches = (*cfg)["branches"];
        if (!branches.is_object()) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "loadControlPipeline: 'branches' must be an object.");
            return false;
        }
        for (auto it = branches.begin(); it != branches.end(); ++it) {
            const std::string& branch_name = it.key();
            const nlohmann::json& stages_json = it.value();
            if (!stages_json.is_array()) {
                FINENAV_LOG_ERROR(node_->get_logger(),
                    "loadControlPipeline: branch '{}' must be an array of stage names.", branch_name);
                ok = false;
                continue;
            }
            std::vector<std::string> stages;
            stages.reserve(stages_json.size());
            for (const auto& s : stages_json) {
                if (!s.is_string()) {
                    FINENAV_LOG_ERROR(node_->get_logger(),
                        "loadControlPipeline: branch '{}' has a non-string stage.", branch_name);
                    ok = false;
                    stages.clear();
                    break;
                }
                stages.push_back(s.get<std::string>());
            }
            if (stages.empty()) {
                if (ok) {   // empty non-error case: addBranch will reject; keep ok = false
                    FINENAV_LOG_ERROR(node_->get_logger(),
                        "loadControlPipeline: branch '{}' is empty.", branch_name);
                    ok = false;
                }
                continue;
            }
            if (!registerControlBranch(branch_name, stages)) ok = false;
        }
    } else {
        FINENAV_LOG_WARN(node_->get_logger(),
            "loadControlPipeline: config '{}' declares no 'branches'.", config_path);
    }

    FINENAV_LOG_INFO(node_->get_logger(),
        "loadControlPipeline: loaded '{}' (ok={}).", config_path, ok);
    return ok;
}

void FineNavEngine::navigateTo(const std::vector<geometry_msgs::msg::Pose>& goals,
                               const std::string& bt_id) {
    const std::string tree_id = bt_id.empty() ? default_bt_id_ : bt_id;
    if (tree_id.empty()) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "navigateTo: behavior tree ID is empty. "
            "Pass a bt_id or call setDefaultBehaviorTree() first.");
        return;
    }

    if (goals.empty()) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "navigateTo: empty goal list — nothing to navigate to.");
        return;
    }

    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "Navigator action server unavailable.");
        return;
    }

    auto goal_msg = NavAction::Goal();
    goal_msg.poses          = goals;
    goal_msg.behavior_tree  = tree_id;

    nav_client_->async_send_goal(goal_msg);
    FINENAV_LOG_INFO(node_->get_logger(),
        "Navigation goal sent: ({:.2f}, {:.2f}, {:.2f})",
        goals.back().position.x, goals.back().position.y, goals.back().position.z);
}

void FineNavEngine::cancelNavigation() {
    nav_client_->async_cancel_all_goals();
    FINENAV_LOG_INFO(node_->get_logger(), "Navigation cancelled.");
}

}  // namespace finenav
