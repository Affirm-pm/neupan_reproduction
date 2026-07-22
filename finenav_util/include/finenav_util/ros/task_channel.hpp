// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/// Master/Worker communication over intra-process ROS topics.
///
/// 3 topics per channel:
///   <name>/__cmd       → TaskCmd      (Master→Worker: start a task)
///   <name>/__stop      → TaskStop     (Master→Worker: stop the current task)
///   <name>/__feedback  → TaskFeedback (Worker→Master: running/completed/failed)

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "finenav_interfaces/msg/task_cmd.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"
#include "finenav_interfaces/msg/task_stop.hpp"

namespace finenav::util {

inline rclcpp::NodeOptions intra_options() {
    auto opts = rclcpp::NodeOptions();
    opts.use_intra_process_comms(true);
    return opts;
}

inline std::shared_ptr<rclcpp::Node> make_intra_node(
    const std::string& name,
    const rclcpp::NodeOptions& extra = rclcpp::NodeOptions()) {
    auto opts = intra_options();
    opts.arguments(extra.arguments());
    return std::make_shared<rclcpp::Node>(name, opts);
}

namespace detail {
constexpr const char* kCmdSuffix      = "/__cmd";
constexpr const char* kStopSuffix     = "/__stop";
constexpr const char* kFeedbackSuffix = "/__feedback";
}  // namespace detail

using CmdMsg      = finenav_interfaces::msg::TaskCmd;
using StopMsg     = finenav_interfaces::msg::TaskStop;
using FeedbackMsg = finenav_interfaces::msg::TaskFeedback;

/**
 * @brief TaskMaster — 1:N Master, manages multiple Workers.
 *
 * Owned by BtNavigator.  Each ControlLayer gets one channel.
 */
class TaskMaster {
public:
    explicit TaskMaster(std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief Register a Worker channel (3 topics). Called once per layer.
     */
    void manageLayer(const std::string& name);
    void unmanageLayer(const std::string& name);

    /**
     * @brief Master → Worker, start a task on a specific layer.
     */
    void start(const std::string& name, CmdMsg&& cmd);

    /**
     * @brief Master → Worker, stop a single layer.
     */
    void stop(const std::string& name);

    /**
     * @brief Master → all Workers, stop every layer.
     */
    void stopAll();

    /**
     * @brief Get latest feedback from a layer (zero-copy shared_ptr pass-through).
     */
    std::shared_ptr<const FeedbackMsg> feedback(const std::string& name) const;

    /**
     * @brief Get all managed layer names.
     */
    std::vector<std::string> layers() const;

private:
    struct Channel {
        std::shared_ptr<rclcpp::Publisher<CmdMsg>>      cmd_pub;
        std::shared_ptr<rclcpp::Publisher<StopMsg>>     stop_pub;
        std::shared_ptr<rclcpp::Subscription<FeedbackMsg>> feedback_sub;
        std::shared_ptr<const FeedbackMsg>             latest_fb;
    };
    Channel& getOrThrow(const std::string& name);
    const Channel& getOrThrow(const std::string& name) const;

    std::shared_ptr<rclcpp::Node> node_;
    std::unordered_map<std::string, Channel> channels_;
};

/**
 * @brief TaskWorker — Worker side
 */
class TaskWorker {
public:
    /**
     * @brief Constructor
     * @param node Shared pointer to a ROS node
     * @param channel_name A specifier for the task channel
     */
    TaskWorker(std::shared_ptr<rclcpp::Node> node,
               const std::string& channel_name);

    /**
     * @brief Worker loop helpers
     * @details
     * hasNewGoal() - Check if a new goal has been received from the master.
     * latestGoal() - Get the latest goal message.
     * shouldStop() - Check if a stop command has been received from the master.
     * ackGoal() - Acknowledge that the latest goal has been processed.
     */
    bool hasNewGoal() const;
    std::shared_ptr<const CmdMsg> latestGoal() const;
    bool shouldStop() const;
    void ackGoal();

    /**
     * @brief Worker -> Master, publish feedback messages
     * @details
     * publishRunning() - Publish a feedback message indicating that the task is currently running.
     * publishCompleted() - Publish a feedback message indicating that the task has been completed successfully.
     * publishFailed() - Publish a feedback message indicating that the task has failed. No trajectories are sent.
     */
    void publishRunning(finenav_interfaces::msg::Trajectory&& traj);
    void publishCompleted(finenav_interfaces::msg::Trajectory&& traj);
    void publishFailed();

private:
    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<rclcpp::Subscription<CmdMsg>>    cmd_sub_;
    std::shared_ptr<rclcpp::Subscription<StopMsg>>   stop_sub_;
    std::shared_ptr<rclcpp::Publisher<FeedbackMsg>>  feedback_pub_;

    std::shared_ptr<const CmdMsg> latest_goal_;
    std::atomic<bool> new_goal_{false};
    std::atomic<bool> should_stop_{false};
};

}  // namespace finenav::util
