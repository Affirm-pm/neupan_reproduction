// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace finenav::localizer {

class EskfLocalizer;

/**
 * @brief SensorSubscriber  —  discovers numbered sensor parameters and creates ROS subscriptions.
 * @details
 *      Probes the node for pose0, pose1, … twist0, twist1, … odom0, odom1, … parameters.
 *      When a non-empty topic is found, reads the per-sensor _config / _queue_size / _gate_dist /
 *      _smoothing_steps overrides and creates the subscription.  Callbacks convert the incoming
 *      message and forward it to the EskfLocalizer.
 *
 *      Odometry sensors support two modes:
 *        mode = "relative" (default) — compute RelPose between consecutive frames
 *        mode = "twist"             — extract twist field and fuse as absolute velocity
 */
class SensorSubscriber {
public:
    using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;
    using OdomMsg = nav_msgs::msg::Odometry;

    /// @param node       The private sub-node (parameter prefix + subscription parent).
    /// @param localizer  The ROS-free ESKF engine.
    SensorSubscriber(rclcpp::Node & node, EskfLocalizer & localizer);

    /// Declare all numbered parameters and create subscriptions.  Call once after construction.
    void discoverAndSubscribe();

    /// Are any sensors configured (i.e. will the localizer receive data once subscribed)?
    [[nodiscard]] bool hasSensors() const;

private:
    // ── Per-sensor config helpers ─────────────────────────────────────────────
    void discoverPoseSensors();
    void discoverTwistSensors();
    void discoverOdomSensors();

    // ── Callbacks (per-sensor index is captured) ──────────────────────────────
    void poseCallback(PoseMsg::ConstSharedPtr msg, std::size_t idx);
    void twistCallback(TwistMsg::ConstSharedPtr msg, std::size_t idx);
    void odomCallback(OdomMsg::ConstSharedPtr msg, std::size_t idx);

    // ── Per-sensor metadata ───────────────────────────────────────────────────
    struct PoseSensor {
        std::string topic;
        int smooth{1};
        double gate_dist{1e9};
        std::array<bool, 6> mask{{true, true, true, true, true, true}};
        std::array<double, 6> cov_override{};  // < 0 → use from msg
        rclcpp::Subscription<PoseMsg>::SharedPtr sub;
    };

    struct TwistSensor {
        std::string topic;
        int smooth{1};
        double gate_dist{1e9};
        std::array<bool, 6> mask{{true, true, true, true, true, true}};
        std::array<double, 6> cov_override{};
        rclcpp::Subscription<TwistMsg>::SharedPtr sub;
    };

    struct OdomSensor {
        std::string topic;
        bool use_relative{true};  // false → twist mode
        int smooth{1};
        double gate_dist{1e9};
        std::array<bool, 6> mask{{true, true, true, true, true, true}};
        std::array<double, 6> cov_override{};
        /// Advanced: also extract selected DOFs from the absolute odom pose field
        /// and fuse them as a Pose observation alongside the primary twist/relative feed.
        std::array<bool, 6> extract_absolute_pose{};
        rclcpp::Subscription<OdomMsg>::SharedPtr sub;
        std::optional<OdomMsg> prev_msg;  // for RelPose mode
    };

    rclcpp::Node &      node_;
    EskfLocalizer &     localizer_;
    rclcpp::Logger      logger_;

    std::vector<PoseSensor>  pose_sensors_;
    std::vector<TwistSensor> twist_sensors_;
    std::vector<OdomSensor>  odom_sensors_;
    rclcpp::Subscription<PoseMsg>::SharedPtr initial_pose_sub_;
};

}  // namespace finenav::localizer
