// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "behavior_tree_engine.hpp"

#include "finenav_util/simple_action_server.hpp"
#include "finenav_util/node_thread.hpp"
#include "finenav_msgs/action/navigate_to_pose.hpp"

#include "finenav_navigator/navigator_params.hpp"

namespace finenav {

/**
 * @brief Navigator — 行为树驱动的任务级导航器。
 *
 * 遵循框架 NodeThread 设计模式：
 *  - 内部持有私有子节点 pnode_（名称固定为 "finenav_navigator"）
 *  - pnode_ 由 NodeThread 在独立线程中 spin，对外部调用方完全透明
 *  - 构造函数完成全部初始化，无需二阶段 initialize()
 *
 * @param parent_node  宿主节点（用于获取 use_sim_time 及命名空间继承）
 */
class Navigator {
public:
    using ActionT      = finenav_msgs::action::NavigateToPose;
    using ActionServer = SimpleActionServer<ActionT>;

    explicit Navigator(rclcpp::Node::SharedPtr parent_node);

    // 禁止复制
    Navigator(const Navigator&)            = delete;
    Navigator& operator=(const Navigator&) = delete;

    ~Navigator() = default;

private:
    void execute();

    // ── ROS 基础设施 ──────────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr       pnode_;
    std::unique_ptr<NodeThread>   pnode_thread_;

    // ── 业务组件 ──────────────────────────────────────────────────────────────
    std::unique_ptr<BehaviorTreeEngine> bt_engine_;
    std::unique_ptr<ActionServer>       action_server_;

    finenav_navigator::Params                    params_;
    std::unique_ptr<finenav_navigator::ParamListener> param_listener_;

    BT::Tree tree_;
};

} // namespace finenav

