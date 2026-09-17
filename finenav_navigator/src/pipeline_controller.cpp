// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/pipeline_controller.hpp"

#include <algorithm>
#include <unordered_set>

#include "finenav_common/frames.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

PipelineController::PipelineController(
    rclcpp::Node::SharedPtr node,
    finenav::core::TaskMaster* master,
    finenav::core::BranchTable* pipe)
    : node_(std::move(node)), master_(master), pipe_(pipe)
{
    root_feedback_pub_ = node_->create_publisher<FeedbackMsg>(
        finenav::core::TaskChannel::feedbackTopic("_root"), 10);
}

void PipelineController::publishNavigationGoal(std::vector<Pose> poses) {
    auto fb = std::make_unique<FeedbackMsg>();
    fb->status = FeedbackMsg::RUNNING;
    fb->trajectory.header.stamp = node_->now();
    fb->trajectory.header.frame_id = common::frames().world_fixed;
    fb->trajectory.valid_fields = finenav_interfaces::msg::Trajectory::TRAJ_POSE;
    fb->trajectory.poses = std::move(poses);
    root_feedback_pub_->publish(std::move(fb));
}

void PipelineController::activateBranch(const std::string& branch_name) {
    if (!pipe_->hasBranch(branch_name)) {
        std::string candidates;
        for (const auto& [name, _] : pipe_->branches()) candidates += " " + name;
        FINENAV_LOG_ERROR(node_->get_logger(),
            "activateBranch: unknown branch '{}'. Known branches:{}", branch_name, candidates);
        return;
    }

    const auto& stages = pipe_->branch(branch_name);   // never contains "_root"
    std::unordered_set<std::string> new_set(stages.begin(), stages.end());

    // 1) 停旧：逆序（下游先停），只停不在新支路里的。
    for (auto it = active_.rbegin(); it != active_.rend(); ++it) {
        if (!new_set.count(*it)) master_->stop(*it);
    }
    // 2) 起新：顺序（上游先起），只起之前没开的。
    for (const auto& s : stages) {
        if (!active_set_.count(s)) master_->start(s);
    }

    active_     = stages;
    active_set_ = std::move(new_set);
}

void PipelineController::stopAll() {
    master_->stopAll();
    active_.clear();
    active_set_.clear();
}

std::shared_ptr<const PipelineController::FeedbackMsg>
PipelineController::feedback(const std::string& name) const {
    return master_->feedback(name);
}

bool PipelineController::hasBranch(const std::string& branch_name) const {
    return pipe_->hasBranch(branch_name);
}

std::vector<std::string> PipelineController::branchNames() const {
    std::vector<std::string> names;
    for (const auto& [name, _] : pipe_->branches()) names.push_back(name);
    return names;
}

std::vector<std::string> PipelineController::branchStages(const std::string& branch_name) const {
    // pipe_->branch(name) 返回 const 引用；未知 branch 返回空 static，拷贝返回。
    return pipe_->branch(branch_name);
}

void PipelineController::stopBranch(const std::string& branch_name) {
    const auto& stages = pipe_->branch(branch_name);   // 未知 branch → 空，自然 no-op
    for (auto it = stages.rbegin(); it != stages.rend(); ++it) {
        master_->stop(*it);
    }
}

void PipelineController::restartBranch(const std::string& branch_name) {
    stopBranch(branch_name);
    const auto& stages = pipe_->branch(branch_name);
    for (const auto& s : stages) {
        master_->start(s);
    }

    // 激活即记账：restartBranch 是 BT 侧唯一的支路激活入口，活跃支路的状态
    // 必须随之更新，否则 restartFrom() 与 BT 的 GetFaultedControlLayer 无从
    // 知道「当前跑的是哪条支路」。
    active_     = stages;
    active_set_ = std::unordered_set<std::string>(stages.begin(), stages.end());
}

void PipelineController::restartFrom(const std::string& control_layer) {
    const auto it = std::find(active_.begin(), active_.end(), control_layer);
    if (it == active_.end()) {
        std::string stages;
        for (const auto& s : active_) stages += " " + s;
        FINENAV_LOG_ERROR(node_->get_logger(),
            "restartFrom: ControlLayer '{}' is not in the active branch (stages[{}]). No-op.",
            control_layer, stages);
        return;
    }

    const std::size_t pos = static_cast<std::size_t>(it - active_.begin());

    // 逆序 STOP 下游后缀（leaf 先停），再顺序 START [control_layer .. leaf]。
    // 上游前缀保持运行 —— 它没有故障，重启它只会白白打断已经正确的产出。
    for (std::size_t k = active_.size(); k-- > pos;) {
        master_->stop(active_[k]);
    }
    for (std::size_t k = pos; k < active_.size(); ++k) {
        master_->start(active_[k]);
    }
}

}  // namespace finenav::navigator
