// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_navigator/behavior_tree_engine.hpp"

#include "finenav_util/ros/simple_action_server.hpp"
#include "finenav_util/ros/node_thread.hpp"
#include "finenav_interfaces/action/navigate_to_pose.hpp"

#include "finenav_navigator/navigator_params.hpp"

namespace finenav::navigator {

/**
 * @brief BtNavigator — 行为树驱动的任务级导航器。
 *
 * 遵循框架 NodeThread 设计模式：
 *  - 内部持有私有子节点 pnode_（名称固定为 "finenav_navigator"）
 *  - pnode_ 由 NodeThread 在独立线程中 spin，对外部调用方完全透明
 *  - 构造函数完成全部初始化，无需二阶段 initialize()
 *
 * @param parent_node    宿主节点（用于获取 use_sim_time 及命名空间继承）
 * @param state_provider 机器人状态提供者（由 FineNavEngine 自动注入）
 */
class BtNavigator {
public:
    using ActionT      = finenav_interfaces::action::NavigateToPose;
    using ActionServer = finenav::util::SimpleActionServer<ActionT>;

    explicit BtNavigator(rclcpp::Node::SharedPtr parent_node,
                       finenav::core::IRobotStateProvider* state_provider = nullptr);

    // 禁止复制
    BtNavigator(const BtNavigator&)            = delete;
    BtNavigator& operator=(const BtNavigator&) = delete;

    ~BtNavigator() = default;

    std::string getActionName() const { return params_.action_name; }

private:
    void execute();

    // ── ROS 基础设施 ──────────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr                     pnode_;
    std::unique_ptr<finenav::util::NodeThread>  pnode_thread_;

    // ── 业务组件 ──────────────────────────────────────────────────────────────
    std::unique_ptr<BehaviorTreeEngine> bt_engine_;
    std::unique_ptr<ActionServer>       action_server_;
    finenav::core::IRobotStateProvider* state_provider_{nullptr};

    finenav_navigator::Params                         params_;
    std::unique_ptr<finenav_navigator::ParamListener> param_listener_;

    BT::Tree tree_;
};

} // namespace finenav::navigator

