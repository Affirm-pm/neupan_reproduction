// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_engine/finenav_engine.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <behaviortree_cpp/utils/shared_library.h>
#include <nlohmann/json.hpp>

#include "finenav_core/plan/plan_registry.hpp"
#include "finenav_core/map/map_registry.hpp"
#include "finenav_core/worldview/world_view_registry.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav {

namespace {

/// Parse a JSON `policy` value into the framework's ExecutionPolicyVariant.
/// Accepted forms (see the design spec §1.1):
///   "event_driven" | "periodic"
///   <number>                                     → PeriodicPolicy{number}
///   {"periodic": hz} | {"frequency_hz": hz}
///   {"type": "periodic"|"event_driven", "frequency_hz": hz}
/// Missing / null / unknown → EventDrivenPolicy (general server default).
finenav::core::ExecutionPolicyVariant parsePolicy(const nlohmann::json& policy) { // TODO: 如果parse逻辑太多，考虑单独拆一个文件
    using finenav::core::PeriodicPolicy;
    using finenav::core::EventDrivenPolicy;

    if (policy.is_null()) return EventDrivenPolicy{};

    auto policy_from_name = [](const std::string& s) -> finenav::core::ExecutionPolicyVariant {
        if (s == "event_driven") return EventDrivenPolicy{};
        if (s == "periodic")     return PeriodicPolicy{};
        return EventDrivenPolicy{};
    };

    if (policy.is_string()) return policy_from_name(policy.get<std::string>());

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
            // unknown type → fall through to the event-driven default below.
        }
        if (have_hz) return PeriodicPolicy{hz};
        // Object with no recognized key → event-driven default.
        return EventDrivenPolicy{};
    }

    // Non-null scalars we don't understand → event-driven default.
    return EventDrivenPolicy{}; // TODO: 不应该静默错误
}

}  // namespace

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
        [this](const geometry_msgs::msg::PoseStamped& msg) { navigateTo({msg.pose}); });
}


bool FineNavEngine::removeMapServer(const std::string& name) {
    return maps_.erase(name) > 0;
}

bool FineNavEngine::removeControlLayer(const std::string& name) {
    return layers_.erase(name) > 0;
}

std::shared_ptr<finenav::core::IPipelineNode> FineNavEngine::createLayerByName(
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

std::shared_ptr<finenav::core::IMapResource> FineNavEngine::ensureMapServer(
    const std::string& algo_name)
{
    auto it = maps_.find(algo_name);
    if (it != maps_.end()) return it->second;
    return createMapServerByName(algo_name, algo_name);
}

std::shared_ptr<finenav::core::IPipelineNode> FineNavEngine::createLayerByWorldView(
    const std::string& world_view_name, const std::string& layer_name,
    finenav::core::ExecutionPolicyVariant policy)
{
    auto entry = finenav::core::WorldViewRegistry::instance().lookup(world_view_name);
    if (!entry) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createLayerByWorldView: WorldView '{}' is not registered. "
            "Call loadAlgoPlugins() first if it lives in a plugin .so.",
            world_view_name);
        return nullptr;
    }

    auto& plans = finenav::core::PlanRegistry::instance();
    auto& maps  = finenav::core::MapRegistry::instance();

    // Resolve the planner type the view bridges → its registered algo name.
    auto planner_name = plans.findPlannerNameByType(entry->planner_type);
    if (!planner_name) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createLayerByWorldView('{}'): planner type '{}' is not registered as an algo.",
            world_view_name, entry->planner_type);
        return nullptr;
    }

    // Resolve + dedup the map server(s) the view depends on.
    std::vector<std::shared_ptr<finenav::core::IMapResource>> map_resources;
    map_resources.reserve(entry->map_types.size());
    for (const auto& map_type : entry->map_types) {
        auto map_name = maps.findMapNameByType(map_type);
        if (!map_name) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "createLayerByWorldView('{}'): map type '{}' is not registered as an algo.",
                world_view_name, map_type);
            return nullptr;
        }
        map_resources.push_back(ensureMapServer(*map_name));
    }

    // 1) create the layer FIRST — its pnode must exist before build
    auto layer = createLayerByName(*planner_name, layer_name, std::move(policy));
    if (!layer) return nullptr;

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
        // createLayerByName(); reclaim it so no orphan layer is left behind.
        removeControlLayer(layer_name);
        FINENAV_LOG_ERROR(node_->get_logger(),
            "createLayerByWorldView('{}'): build() returned no view.", world_view_name);
        return nullptr;
    }

    // 4) attach the view + its param pollable + its prepare() invoker
    layer->attachWorldView(std::move(built), std::move(map_resources));
    FINENAV_LOG_INFO(node_->get_logger(),
        "createLayerByWorldView('{}', '{}'): assembled layer over planner '{}'.",
        world_view_name, layer_name, *planner_name);
    return layer;
}

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

void FineNavEngine::wireBranch(const std::string& branch_name,
                               const std::vector<std::string>& stages) {
    if (stages.empty()) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "wireBranch('{}'): branch carries no stages.", branch_name);
        return;
    }
    // Every stage must already exist as a layer (created by a layer factory or
    // by loadControlPipeline()'s layer pass).
    for (const auto& s : stages) {
        if (!layers_.count(s)) {
            FINENAV_LOG_ERROR(node_->get_logger(),
                "wireBranch('{}'): stage '{}' is not a created layer.", branch_name, s);
            return;
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
            "wireBranch('{}'): addBranch rejected (duplicate branch name, empty "
            "stages, duplicate stage, or '_root' inside stages).", branch_name);
    }
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
                policy = parsePolicy(spec["policy"]);
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

            std::shared_ptr<finenav::core::IPipelineNode> node;
            if (!world_view.empty()) {
                node = createLayerByWorldView(world_view, layer_name, std::move(policy));
            } else if (!algo.empty()) {
                node = createLayerByName(algo, layer_name, std::move(policy));
            }
            if (!node) {
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
            wireBranch(branch_name, stages);
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
