// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/behavior_tree_engine.hpp"

#include <behaviortree_cpp/utils/shared_library.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

BehaviorTreeEngine::BehaviorTreeEngine() = default;

void BehaviorTreeEngine::registerPlugins(const std::vector<std::string>& plugins)
{
    FINENAV_LOG_INFO(rclcpp::get_logger("BehaviorTreeEngine"),
        "Loading plugins from {} sources...", plugins.size());

    for (const auto& plugin_param : plugins) {
        const auto [pkg_name, sub_folder] = parseParamPath(plugin_param);
        if (pkg_name.empty()) continue;

        std::filesystem::path plugin_directory;
        try {
            const auto prefix = ament_index_cpp::get_package_prefix(pkg_name);
            plugin_directory = std::filesystem::path(prefix) / sub_folder;
        }
        catch (const ament_index_cpp::PackageNotFoundError& e) {
            FINENAV_LOG_ERROR(rclcpp::get_logger("BehaviorTreeEngine"),
                "Package '{}' not found.", pkg_name);
            continue;
        }
        if (!std::filesystem::exists(plugin_directory)) {
            FINENAV_LOG_WARN(rclcpp::get_logger("BehaviorTreeEngine"),
                "Plugin directory not found: {}", plugin_directory.string());
            continue;
        }

        int loaded_count = 0;
        for (const auto& entry :
             std::filesystem::recursive_directory_iterator(plugin_directory)) {
            if (entry.path().extension() == ".so") {
                if (loadPlugin(factory_, entry.path())) loaded_count++;
            }
        }
        FINENAV_LOG_INFO(rclcpp::get_logger("BehaviorTreeEngine"),
            "  Scanned '{}': Loaded {} plugins.",
            plugin_directory.string(), loaded_count);
    }
}

void BehaviorTreeEngine::registerBehaviorTrees(const std::vector<std::string>& tree_dirs)
{
    for (const auto& tree_dir_param : tree_dirs) {
        const auto [pkg_name, sub_folder] = parseParamPath(tree_dir_param);
        if (pkg_name.empty()) continue;

        std::filesystem::path tree_directory;
        try {
            const auto share_dir = ament_index_cpp::get_package_share_directory(pkg_name);
            tree_directory = std::filesystem::path(share_dir) / sub_folder;
        }
        catch (const ament_index_cpp::PackageNotFoundError& e) {
            FINENAV_LOG_ERROR(rclcpp::get_logger("BehaviorTreeEngine"),
                "Package '{}' not found.", pkg_name);
            continue;
        }
        if (!std::filesystem::exists(tree_directory)) {
            FINENAV_LOG_WARN(rclcpp::get_logger("BehaviorTreeEngine"),
                "Directory not found: {}", tree_directory.string());
            continue;
        }

        int loaded_count = 0;
        for (const auto& entry :
             std::filesystem::recursive_directory_iterator(tree_directory)) {
            if (loadBehaviorTree(factory_, entry.path())) loaded_count++;
        }
        FINENAV_LOG_INFO(rclcpp::get_logger("BehaviorTreeEngine"),
            "  Scanned '{}': Loaded {} trees.",
            tree_directory.string(), loaded_count);
    }
}

bool BehaviorTreeEngine::loadPlugin(
    BT::BehaviorTreeFactory& factory,
    const std::filesystem::path& file_path)
{
    const auto filename = file_path.filename();
    try {
        BT::SharedLibrary loader(file_path.string());
        if (loader.hasSymbol(BT::PLUGIN_SYMBOL)) {
            typedef void (*Func)(BT::BehaviorTreeFactory&);
            auto func = (Func)loader.getSymbol(BT::PLUGIN_SYMBOL);
            func(factory);
        } else {
            FINENAV_LOG_ERROR(rclcpp::get_logger("BehaviorTreeEngine"),
                "No plugin symbol in file: {}", filename.string());
            return false;
        }
        return true;
    } catch (const std::exception& ex) {
        FINENAV_LOG_ERROR(rclcpp::get_logger("BehaviorTreeEngine"),
            "Failed to load plugin {}: {}", filename.string(), ex.what());
        return false;
    }
}

bool BehaviorTreeEngine::loadBehaviorTree(
    BT::BehaviorTreeFactory& factory,
    const std::filesystem::path& file_path)
{
    try {
        factory.registerBehaviorTreeFromFile(file_path.string());
        return true;
    } catch (const std::exception& e) {
        FINENAV_LOG_ERROR(rclcpp::get_logger("BehaviorTreeEngine"),
            "Failed to load behavior tree {}: {}",
            file_path.filename().string(), e.what());
        return false;
    }
}

std::pair<std::string, std::string>
BehaviorTreeEngine::parseParamPath(const std::string& input) {
    auto pos = input.find('/');
    if (pos == std::string::npos) return {input, ""};
    return {input.substr(0, pos), input.substr(pos + 1)};
}

}  // namespace finenav::navigator
