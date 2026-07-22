// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/behavior_tree_engine.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

BehaviorTreeEngine::BehaviorTreeEngine(rclcpp::Node::SharedPtr node)
  : logger_(node->get_logger())
{
}

void BehaviorTreeEngine::registerPlugins(const std::vector<std::string>& plugins)
{

    FINENAV_LOG_INFO(logger_, "FineNav Engine: Loading plugins from {} sources...", plugins.size());

    for (const auto& plugin_param : plugins) {
        const auto [pkg_name, sub_folder] = parseParamPath(plugin_param);
        if (pkg_name.empty()) { continue; } // skip invalid plugins directories

        std::filesystem::path plugin_directory;
        try {
            // Plugins are usually installed in "[ws]/install/[pkg_name]/lib/"
            const auto prefix = ament_index_cpp::get_package_prefix(pkg_name);
            plugin_directory = std::filesystem::path(prefix) / sub_folder;
        }
        catch (const ament_index_cpp::PackageNotFoundError& e) {
            FINENAV_LOG_ERROR(logger_, "Plugin loading failed: Package '{}' not found.", pkg_name);
            continue;
        }
        if (!std::filesystem::exists(plugin_directory)) {
            FINENAV_LOG_WARN(logger_, "Plugin directory not found: {}", plugin_directory.string());
            continue;
        }

        // Discovery and load .so
        using std::filesystem::recursive_directory_iterator;
        int loaded_count = 0;
        for(const auto& entry : recursive_directory_iterator(plugin_directory))
        {
            if(entry.path().extension() == ".so")
            {
                if (loadPlugin(factory_, entry.path(), ros_params_)) {
                    loaded_count++;
                }
            }
        }
        FINENAV_LOG_INFO(logger_, "  -> Scanned '{}': Loaded {} plugins.", plugin_directory.string(), loaded_count);
    }
}

void BehaviorTreeEngine::registerBehaviorTrees(const std::vector<std::string>& tree_dirs)
{
    for (const auto& tree_dir_param : tree_dirs) {
        const auto [pkg_name, sub_folder] = parseParamPath(tree_dir_param);
        if (pkg_name.empty()) { continue; } // skip invalid plugins directories

        std::filesystem::path tree_directory;
        try {
            // BehaviorTress are usually installed in "[ws]/install/[pkg_name]/share/[pkg_name]/"
            const auto share_dir = ament_index_cpp::get_package_share_directory(pkg_name);
            tree_directory = std::filesystem::path(share_dir) / sub_folder;
        }
        catch (const ament_index_cpp::PackageNotFoundError& e) {
            FINENAV_LOG_ERROR(logger_, "BT loading failed: Package '{}' not found.", pkg_name);
            continue;
        }
        if (!std::filesystem::exists(tree_directory)) {
            FINENAV_LOG_WARN(logger_, "Plugin directory not found: {}", tree_directory.string());
            continue;
        }

        // Discovery and load .xml
        using std::filesystem::recursive_directory_iterator;
        int loaded_count = 0;
        for(const auto& entry : recursive_directory_iterator(tree_directory))
        {
            if (loadBehaviorTree(factory_, entry.path())) {
                loaded_count++;
            }
        }
        FINENAV_LOG_INFO(logger_, "  -> Scanned '{}': Loaded {} trees.", tree_directory.string(), loaded_count);
    }
}

bool BehaviorTreeEngine::loadPlugin(
    BT::BehaviorTreeFactory& factory,
    const std::filesystem::path& file_path,
    BT::RosNodeParams params) {

    const auto filename = file_path.filename();
    try
    {
        BT::SharedLibrary loader(file_path.string());
        if(loader.hasSymbol(BT::PLUGIN_SYMBOL))
        {
            typedef void (*Func)(BT::BehaviorTreeFactory&);
            auto func = (Func)loader.getSymbol(BT::PLUGIN_SYMBOL);
            func(factory);
        }
        else if(loader.hasSymbol(BT::ROS_PLUGIN_SYMBOL))
        {
            typedef void (*Func)(BT::BehaviorTreeFactory&, const BT::RosNodeParams&);
            auto func = (Func)loader.getSymbol(BT::ROS_PLUGIN_SYMBOL);
            func(factory, params);
        }
        else
        {
            FINENAV_LOG_ERROR(logger_, "Failed to load Plugin from file: {}.", filename.string());
            return false;
        }
        FINENAV_LOG_INFO(logger_, "Loaded ROS Plugin: {}", filename.string());
        return true;
    }
    catch(const std::exception& ex)
    {
        FINENAV_LOG_ERROR(logger_, "Failed to load ROS Plugin: {} \n {}", filename.string(),
                     ex.what());
        return false;
    }
}

bool BehaviorTreeEngine::loadBehaviorTree(
    BT::BehaviorTreeFactory& factory,
    const std::filesystem::path& file_path) {

    try
    {
        factory.registerBehaviorTreeFromFile(file_path.string());
        FINENAV_LOG_INFO(logger_, "Loaded BehaviorTree: {}", file_path.filename().string());
        return true;
    }
    catch(const std::exception& e)
    {
        FINENAV_LOG_ERROR(logger_, "Failed to load BehaviorTree: {} \n {}",
                     file_path.filename().string(), e.what());
        return false;
    }
}

BT::BehaviorTreeFactory& BehaviorTreeEngine::factory() {
    return factory_;
}

std::pair<std::string, std::string> BehaviorTreeEngine::parseParamPath(const std::string& input) {
    auto pos = input.find('/');
    if (pos == std::string::npos) {
        return {input, ""}; // only package name
    }
    return {input.substr(0, pos), input.substr(pos + 1)};
}

}
