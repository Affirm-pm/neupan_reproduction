// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_util/ros/task_channel.hpp"

namespace finenav::util {

// ══════════════════════════════════════════════════════════════════════════════
//  TaskMaster
// ══════════════════════════════════════════════════════════════════════════════

TaskMaster::TaskMaster(std::shared_ptr<rclcpp::Node> node)
    : node_(std::move(node)) {}

void TaskMaster::manageLayer(const std::string& name) {
    if (channels_.count(name)) return;  // already registered

    Channel ch;
    ch.cmd_pub  = node_->create_publisher<CmdMsg>(TaskChannel::cmdTopic(name), 10);
    ch.stop_pub = node_->create_publisher<StopMsg>(TaskChannel::stopTopic(name), 10);
    ch.feedback_sub = node_->create_subscription<FeedbackMsg>(
        TaskChannel::feedbackTopic(name), 10,
        [this, name](std::shared_ptr<const FeedbackMsg> msg) {
            auto it = channels_.find(name);
            if (it != channels_.end())
                std::atomic_store(&it->second.latest_fb, std::move(msg));
        });
    channels_.emplace(name, std::move(ch));
}

void TaskMaster::unmanageLayer(const std::string& name) {
    channels_.erase(name);
}

void TaskMaster::start(const std::string& name, CmdMsg&& cmd) {
    auto& ch = getOrThrow(name);
    ch.cmd_pub->publish(std::make_unique<CmdMsg>(std::move(cmd)));
}

void TaskMaster::stop(const std::string& name) {
    auto& ch = getOrThrow(name);
    ch.stop_pub->publish(std::make_unique<StopMsg>());
}

void TaskMaster::stopAll() {
    for (auto& [name, ch] : channels_) {
        ch.stop_pub->publish(std::make_unique<StopMsg>());
    }
}

std::shared_ptr<const FeedbackMsg> TaskMaster::feedback(const std::string& name) const {
    return std::atomic_load(&getOrThrow(name).latest_fb);
}

std::vector<std::string> TaskMaster::layers() const {
    std::vector<std::string> names;
    for (const auto& [name, _] : channels_) names.push_back(name);
    return names;
}

TaskMaster::Channel& TaskMaster::getOrThrow(const std::string& name) {
    auto it = channels_.find(name);
    if (it == channels_.end())
        throw std::runtime_error("TaskMaster: unmanaged layer '" + name + "'");
    return it->second;
}

const TaskMaster::Channel& TaskMaster::getOrThrow(const std::string& name) const {
    auto it = channels_.find(name);
    if (it == channels_.end())
        throw std::runtime_error("TaskMaster: unmanaged layer '" + name + "'");
    return it->second;
}

// ══════════════════════════════════════════════════════════════════════════════
//  TaskWorker
// ══════════════════════════════════════════════════════════════════════════════

TaskWorker::TaskWorker(std::shared_ptr<rclcpp::Node> node,
                       const std::string& channel_name)
    : node_(std::move(node)) {
    feedback_pub_ = node_->create_publisher<FeedbackMsg>(
        TaskChannel::feedbackTopic(channel_name), 10);

    cmd_sub_ = node_->create_subscription<CmdMsg>(
        TaskChannel::cmdTopic(channel_name), 10,
        [this](std::shared_ptr<const CmdMsg> msg) {
            latest_goal_ = std::move(msg);
            new_goal_.store(true, std::memory_order_release);
        });

    stop_sub_ = node_->create_subscription<StopMsg>(
        TaskChannel::stopTopic(channel_name), 10,
        [this](std::shared_ptr<const StopMsg> /*msg*/) {
            should_stop_.store(true, std::memory_order_release);
        });
}

bool TaskWorker::hasNewGoal() const {
    return new_goal_.load(std::memory_order_acquire);
}

std::shared_ptr<const CmdMsg> TaskWorker::latestGoal() const {
    return latest_goal_;
}

bool TaskWorker::shouldStop() const {
    return should_stop_.load(std::memory_order_acquire);
}

void TaskWorker::ackGoal() {
    new_goal_.store(false, std::memory_order_release);
    // A Start naturally supersedes any previous Stop — the worker is
    // about to begin a new episode.
    should_stop_.store(false, std::memory_order_release);
}

void TaskWorker::publishRunning(finenav_interfaces::msg::Trajectory&& traj) {
    auto msg        = std::make_unique<FeedbackMsg>();
    msg->status     = FeedbackMsg::RUNNING;
    msg->trajectory = std::move(traj);
    feedback_pub_->publish(std::move(msg));
}

void TaskWorker::publishCompleted(finenav_interfaces::msg::Trajectory&& traj) {
    auto msg        = std::make_unique<FeedbackMsg>();
    msg->status     = FeedbackMsg::COMPLETED;
    msg->trajectory = std::move(traj);
    feedback_pub_->publish(std::move(msg));
}

void TaskWorker::publishFailed() {
    auto msg     = std::make_unique<FeedbackMsg>();
    msg->status  = FeedbackMsg::FAILED;
    feedback_pub_->publish(std::move(msg));
}

}  // namespace finenav::util
