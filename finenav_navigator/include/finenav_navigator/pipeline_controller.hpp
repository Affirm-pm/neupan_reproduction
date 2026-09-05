// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "finenav_core/plan/control_pipeline_tree.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"
#include "finenav_util/ros/task_channel.hpp"

namespace finenav::navigator {

/**
 * @brief PipelineController — 统一封装对 control_pipeline 的控制与数据操作。
 *
 * 持有 TaskMaster（启停层）与 ControlPipelineTree（拓扑），并暴露 _root 目标注入
 * （开始导航）。BT 节点通过它操作流水线，不再直接触碰 TaskMaster / _root topic。
 */
class PipelineController {
public:
    using FeedbackMsg = finenav_interfaces::msg::TaskFeedback;
    using Pose = geometry_msgs::msg::Pose;

    PipelineController(rclcpp::Node::SharedPtr node,
                       finenav::util::TaskMaster* master,
                       finenav::core::ControlPipelineTree* pipe);

    /// 注入导航目标：把 goal_poses[] 发布到 _root/__feedback（开始导航）。
    void publishNavigationGoal(std::vector<Pose> poses);

    /// 一键启动支路：停 off-path 层，再按拓扑序（root→leaf）启动 on-path 层。
    void startBranch(const std::string& to);

    /// 启动 / 停止单个层。
    void startLayer(const std::string& name);
    void stopLayer(const std::string& name);
    void stopAll();

    /// 取某层最新反馈。
    std::shared_ptr<const FeedbackMsg> feedback(const std::string& name) const;

    /// _root → ... → to 的唯一路径（不含 _root），root→leaf 顺序。
    std::vector<std::string> pathTo(const std::string& to) const;

    /// 所有 leaf（out-degree == 0）。
    std::vector<std::string> leaves() const;

private:
    rclcpp::Node::SharedPtr node_;
    finenav::util::TaskMaster* master_;
    finenav::core::ControlPipelineTree* pipe_;
    rclcpp::Publisher<FeedbackMsg>::SharedPtr root_feedback_pub_;
};

}  // namespace finenav::navigator
