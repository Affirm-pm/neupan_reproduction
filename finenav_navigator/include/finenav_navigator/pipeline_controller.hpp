// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "finenav_core/plan/branch_table.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"
#include "finenav_core/task_bus/task_bus.hpp"

namespace finenav::navigator {

/**
 * @brief PipelineController — 统一封装对 control_pipeline 的控制与数据操作。
 *
 * 持有 TaskMaster（启停层）与 BranchTable（支路表拓扑），并暴露 _root 目标注入
 * （开始导航）。BT 节点通过它操作流水线，不再直接触碰 TaskMaster / _root topic。
 *
 * 运行时唯一激活入口是 activateBranch()：按 **集合差** 切换活跃支路（见 §2.1
 * 设计规格）。逐层启停自由度已删除，只保留支路级激活与整管清停。
 */
class PipelineController {
public:
    using FeedbackMsg = finenav_interfaces::msg::TaskFeedback;
    using Pose = geometry_msgs::msg::Pose;

    PipelineController(rclcpp::Node::SharedPtr node,
                       finenav::core::TaskMaster* master,
                       finenav::core::BranchTable* pipe);

    /// 注入导航目标：把 goal_poses[] 发布到 _root/__feedback（开始导航）。
    void publishNavigationGoal(std::vector<Pose> poses);

    /// 集合差激活一条支路：停「旧−新」（逆序，下游先停）、起「新−旧」（顺序，
    /// 上游先起）、共享 stage 保持运行。未知支路名 → ERROR 日志 + no-op。
    void activateBranch(const std::string& branch_name);

    /// 整管清停（取消 / 抢占 / 成功 / 失败时），并清空活跃支路状态。
    void stopAll();

    /// 取某层最新反馈。
    std::shared_ptr<const FeedbackMsg> feedback(const std::string& name) const;

    /// 支路表只读查询（BT 节点用：未知支路 fail loudly）。
    bool hasBranch(const std::string& branch_name) const;
    std::vector<std::string> branchNames() const;

private:
    rclcpp::Node::SharedPtr node_;
    finenav::core::TaskMaster* master_;
    finenav::core::BranchTable* pipe_;
    rclcpp::Publisher<FeedbackMsg>::SharedPtr root_feedback_pub_;

    /// 当前活跃支路的 stage 序列 / 集合（Mux 状态）。
    std::vector<std::string> active_;
    std::unordered_set<std::string> active_set_;
};

}  // namespace finenav::navigator
