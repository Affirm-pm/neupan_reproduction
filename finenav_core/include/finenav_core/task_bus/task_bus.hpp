// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/// Master/Worker communication over intra-process ROS topics.
///
/// 2 topics per channel:
///   <name>/__cmd       → TaskCmd      (Master→Worker: START/STOP control)
///   <name>/__feedback  → TaskFeedback (Worker→Master: idle/running/stopped/faulted)

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "finenav_interfaces/msg/task_cmd.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"

namespace finenav::core {

struct TaskChannel {
    static std::string cmdTopic(const std::string& name)      { return name + "/__cmd"; }
    static std::string feedbackTopic(const std::string& name) { return name + "/__feedback"; }
};

using CmdMsg      = finenav_interfaces::msg::TaskCmd;
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
     * @brief Register a Worker channel (2 topics). Called once per layer.
     */
    void manageLayer(const std::string& name);
    void unmanageLayer(const std::string& name);

    /**
     * @brief Master → Worker, activate a layer for the next episode.
     */
    void start(const std::string& name);

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
     * @brief Register a possible predecessor's __feedback as a reference input.
     *
     * Idempotent: a second connectFrom() for the same @p parent_name is a no-op
     * (the subscription already exists).  Different parent names each get their
     * own subscription (fan-out) — the worker consumes whichever upstream is
     * publishing on the ACTIVE branch.
     * @param parent_name  Name of a possible upstream layer.
     * @param on_ref       Invoked with each new reference (zero-copy shared_ptr).
     */
    void connectFrom(const std::string& parent_name,
                     std::function<void(std::shared_ptr<const FeedbackMsg>)> on_ref);

    /**
     * @brief Worker loop helpers
     * @details
     * hasStart()  - Check if a START control has been received from the master.
     * shouldStop()- Check if a STOP control has been received from the master.
     * ackStart()  - Acknowledge that the START has been processed (also clears
     *               any pending STOP: a START supersedes a prior STOP).
     */
    bool hasStart() const;
    bool shouldStop() const;
    void ackStart();

    /**
     * @brief Worker -> Master, publish feedback messages
     * @details
     * publishIdle()    - Publish IDLE: layer created / not yet STARTed (the safe at-rest state).
     * publishRunning() - Publish RUNNING: STARTed, actively planning. Carries a trajectory.
     * publishStopped() - Publish STOPPED: received STOP, normal termination. No trajectory.
     * publishFaulted() - Publish FAULTED: plan() returned nullopt, abnormal termination. No trajectory.
     */
    void publishIdle();
    void publishRunning(finenav_interfaces::msg::Trajectory&& traj);
    void publishStopped();
    void publishFaulted();

private:
    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<rclcpp::Subscription<CmdMsg>>    cmd_sub_;
    // One subscription per possible predecessor (fan-out): a shared stage may
    // be fed by different upstreams on different branches.  Only the upstream
    // of the ACTIVE branch publishes, so at most one ref is live at a time
    // (last-writer-wins switches when the active branch changes).
    std::unordered_map<std::string, std::shared_ptr<rclcpp::Subscription<FeedbackMsg>>> ref_subs_;
    std::shared_ptr<rclcpp::Publisher<FeedbackMsg>>  feedback_pub_;

    std::atomic<bool> start_{false};
    std::atomic<bool> should_stop_{false};
};

}  // namespace finenav::core
