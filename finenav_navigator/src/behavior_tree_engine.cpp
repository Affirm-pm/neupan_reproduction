// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "behavior_tree_engine.hpp"
#include "behaviortree_ros2/bt_utils.hpp"

namespace finenav {

BehaviorTreeEngine::BehaviorTreeEngine(rclcpp::Node::SharedPtr node)
  : node_(node)
{
    // 初始化 RosNodeParams，这是 BT.ROS2 依赖注入的核心
    ros_params_.nh = node;
    ros_params_.default_port_value = ""; // 默认不指定 topic，强制 XML 指定
    ros_params_.server_timeout = std::chrono::milliseconds(1000);
    ros_params_.wait_for_server_timeout = std::chrono::milliseconds(500);
}

void BehaviorTreeEngine::registerPlugins(const std::vector<std::string>& plugins)
{
    auto node_shared = node_.lock(); // TODO: 这在干嘛
    if (!node_shared) {
        throw std::runtime_error("BehaviorTreeEngine: Node has expired!");
    }

    RCLCPP_INFO(node_shared->get_logger(), "FineNav Engine: Loading %zu plugins...", plugins.size());

    for (const auto& plugin : plugins) {
        const auto plugin_directory = BT::GetDirectoryPath(plugin);
        if (plugin_directory.empty()) { continue; } // skip invalid plugins directories

        using std::filesystem::directory_iterator;

        for(const auto& entry : directory_iterator(plugin_directory))
        {
            if(entry.path().extension() == ".so")
            {
                BT::LoadPlugin(factory_, entry.path(), ros_params_);
            }
        }
    }
}

void BehaviorTreeEngine::registerBehaviorTrees(const std::vector<std::string>& tree_dirs)
{
    auto node_shared = node_.lock();
    if (!node_shared) return;

    for (const auto& tree_dir : tree_dirs) {
        const auto tree_directory = BT::GetDirectoryPath(tree_dir);
        if (tree_directory.empty()) { continue; } // skip invalid plugins directories
        BT::LoadBehaviorTrees(factory_, tree_directory);
    }
}

BT::BehaviorTreeFactory& BehaviorTreeEngine::factory() {
    return factory_;
}


}
