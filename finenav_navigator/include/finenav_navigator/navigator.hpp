// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_core/plan/control_pipeline_tree.hpp"
#include "finenav_navigator/behavior_tree_engine.hpp"
#include "finenav_navigator/chassis_command/i_chassis_command.hpp"
#include "finenav_navigator/pipeline_controller.hpp"

#include "finenav_core/task_bus/task_bus.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"
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
                       finenav::core::RobotStateProvider* state_provider = nullptr);

    // 禁止复制
    BtNavigator(const BtNavigator&)            = delete;
    BtNavigator& operator=(const BtNavigator&) = delete;

    ~BtNavigator() = default;

    /// Return the fully-qualified action name so clients on any node can reach the server.
    std::string getActionName() const {
        return std::string(pnode_->get_namespace()) + "/" + params_.action_name;
    }

    // ── IPC + topology (public for FineNavEngine wiring) ────────────────────
    finenav::core::TaskMaster&       master()       { return *master_; } // TODO: 如果没人用，就去掉
    finenav::core::ControlPipelineTree& pipe()       { return pipe_; }

    void setChassisCommand(std::shared_ptr<IChassisCommand> cmd) {
        chassis_cmd_ = std::move(cmd);
    }

private:
    void execute();

    // ── ROS 基础设施 ──────────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr                     parent_node_;
    rclcpp::Node::SharedPtr                     pnode_;
    std::unique_ptr<finenav::util::NodeThread>  pnode_thread_;

    // ── 业务组件 ──────────────────────────────────────────────────────────────
    std::unique_ptr<BehaviorTreeEngine> bt_engine_;
    std::unique_ptr<ActionServer>       action_server_;
    finenav::core::RobotStateProvider* state_provider_{nullptr};

    finenav_navigator::Params                         params_;
    std::unique_ptr<finenav_navigator::ParamListener> param_listener_;

    std::unique_ptr<finenav::core::TaskMaster> master_;
    finenav::core::ControlPipelineTree pipe_;
    std::unique_ptr<PipelineController> pipeline_;
    using FeedbackMsg = finenav_interfaces::msg::TaskFeedback;

    std::shared_ptr<IChassisCommand>   chassis_cmd_;
    std::unordered_map<std::string, std::shared_ptr<const FeedbackMsg>> last_consumed_fb_;
    rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};

    BT::Tree tree_;
};

} // namespace finenav::navigator

