// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/pipeline_controller.hpp"

#include <algorithm>
#include <unordered_set>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::navigator {

PipelineController::PipelineController(
    rclcpp::Node::SharedPtr node,
    finenav::util::TaskMaster* master,
    finenav::core::ControlPipelineTree* pipe)
    : node_(std::move(node)), master_(master), pipe_(pipe)
{
    root_feedback_pub_ = node_->create_publisher<FeedbackMsg>(
        finenav::util::TaskChannel::feedbackTopic("_root"), 10);
}

void PipelineController::publishNavigationGoal(const std::vector<Pose>& poses) {
    FeedbackMsg fb;
    fb.status = FeedbackMsg::RUNNING;
    fb.trajectory.header.stamp = node_->now();
    fb.trajectory.header.frame_id = "map";
    fb.trajectory.valid_fields = finenav_interfaces::msg::Trajectory::TRAJ_POSE;
    fb.trajectory.poses = poses;
    root_feedback_pub_->publish(fb);
}

void PipelineController::startBranch(const std::string& to) {
    if (!pipe_->hasNode(to)) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "startBranch: '{}' is not a node in the control pipeline.", to);
        return;
    }

    const auto path = pathTo(to);
    std::unordered_set<std::string> on_path(path.begin(), path.end());

    // 1) 停 off-path 层（维护 mux 不变量：任意时刻至多一条 root→leaf 链活跃）。
    for (const auto& name : pipe_->nodes()) {
        if (!on_path.count(name)) master_->stop(name);
    }
    // 2) 按拓扑序（root→leaf）启动 on-path 层。
    for (const auto& name : path) master_->start(name);
}

void PipelineController::startLayer(const std::string& name) { master_->start(name); }
void PipelineController::stopLayer(const std::string& name)  { master_->stop(name); }
void PipelineController::stopAll()                            { master_->stopAll(); }

std::shared_ptr<const PipelineController::FeedbackMsg>
PipelineController::feedback(const std::string& name) const {
    return master_->feedback(name);
}

std::vector<std::string> PipelineController::pathTo(const std::string& to) const {
    std::vector<std::string> path;
    std::string cur = to;
    while (!cur.empty() && cur != "_root") {
        path.push_back(cur);
        cur = pipe_->parent(cur);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::string> PipelineController::leaves() const {
    return pipe_->leaves();
}

}  // namespace finenav::navigator
