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
#include <mutex>
#include <string>

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
 * @brief TaskCommander — Master side
 */
class TaskCommander {
public:
    /**
     * @brief Constructor
     * @param node Shared pointer to a ROS node
     * @param channel_name A specifier for the task channel
     */
    TaskCommander(std::shared_ptr<rclcpp::Node> node,
                  const std::string& channel_name);
    /**
     * @brief Master → Worker, start a task
     * @param cmd TaskCmd message
     */
    void start(const CmdMsg& cmd);

    /**
     * @brief Master -> Worker, stop the current task (if any)
     */
    void stop();

    /**
     * @brief Get the latest feedback from the worker.
     * @note Thread-safe
     */
    FeedbackMsg latestFeedback() const;

    /**
     * @brief Register a callback to receive feedback messages from the worker.
     * @param cb Callback function
     */
    void onFeedback(std::function<void(const FeedbackMsg&)> cb);

    /**
     * @brief Get the channel name used for this TaskCommander.
     */
    const std::string& channel() const { return channel_; }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string channel_;

    std::shared_ptr<rclcpp::Publisher<CmdMsg>>      cmd_pub_;
    std::shared_ptr<rclcpp::Publisher<StopMsg>>     stop_pub_;
    std::shared_ptr<rclcpp::Subscription<FeedbackMsg>> feedback_sub_;

    mutable std::mutex fb_mutex_;
    FeedbackMsg latest_feedback_;
    std::function<void(const FeedbackMsg&)> on_feedback_;
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
    CmdMsg latestGoal() const;
    bool shouldStop() const;
    void ackGoal();

    /**
     * @brief Worker -> Master, publish feedback messages
     * @details
     * publishRunning() - Publish a feedback message indicating that the task is currently running.
     * publishCompleted() - Publish a feedback message indicating that the task has been completed successfully.
     * publishFailed() - Publish a feedback message indicating that the task has failed. No trajectories are sent.
     */
    void publishRunning(const finenav_interfaces::msg::Trajectory& traj);
    void publishCompleted(const finenav_interfaces::msg::Trajectory& traj);
    void publishFailed();

private:
    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<rclcpp::Subscription<CmdMsg>>    cmd_sub_;
    std::shared_ptr<rclcpp::Subscription<StopMsg>>   stop_sub_;
    std::shared_ptr<rclcpp::Publisher<FeedbackMsg>>  feedback_pub_;

    mutable std::mutex goal_mutex_;
    CmdMsg latest_goal_;
    std::atomic<bool> new_goal_{false};
    std::atomic<bool> should_stop_{false};
};

}  // namespace finenav::util
