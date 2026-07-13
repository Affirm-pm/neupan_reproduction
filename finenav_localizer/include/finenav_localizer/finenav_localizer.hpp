// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_localizer/eskf_localizer.hpp"
#include "finenav_util/ros/node_thread.hpp"
#include "finenav_localizer/localizer_params.hpp"

namespace finenav::localizer {

/**
 * @brief FineNavLocalizer  —  ROS wrapper around the EskfLocalizer engine
 * @details
 *      The outer half of the localizer.  Owns the ROS runtime (a private sub-node,
 *      spin thread, periodic timer, parameter listener and TF broadcaster) and
 *      drives the inner EskfLocalizer (the ROS-free C++ engine):
 *        • each timer tick: refresh params → setConfig → step → broadcast TF →
 *          overrun check,
 *        • is a RobotStateProvider: each tick it converts the engine's latest state
 *          to RobotState and pushState()s into the inherited history;
 *          getRobotState()/(t) are served by the base.
 *
 *      Input side (ROS subscriptions to pose / twist / odom / initial-pose topics)
 *      is intentionally NOT implemented here yet — see the TODO below.  For direct
 *      C++ use (benchmarks / tests / offline eval), construct and drive EskfLocalizer
 *      directly.
 */
class FineNavLocalizer : public finenav::core::RobotStateProvider {
public:
    using PMat          = EskfLocalizer::PMat;
    using Params        = finenav_localizer::Params;
    using ParamListener = finenav_localizer::ParamListener;

    /**
     * @brief Construct the ROS wrapper.  Creates a private "finenav_localizer"
     *        sub-node, reads parameters, builds the inner engine, and starts the
     *        update timer + spin thread.
     */
    explicit FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                              rclcpp::NodeOptions pnode_overrides = rclcpp::NodeOptions{});

    FineNavLocalizer(const FineNavLocalizer &)             = delete;
    FineNavLocalizer & operator=(const FineNavLocalizer &) = delete;
    ~FineNavLocalizer() = default;

    //  getRobotState() / getRobotState(t) are inherited from RobotStateProvider
    //  (history-backed); this wrapper feeds the history via pushState() each tick.

    /// TODO: ROS message-subscription input side (future work).
    ///   This wrapper will own the localizer's ROS inputs: subscriptions to the
    ///   initial-pose, pose, twist and odometry topics.  Each callback converts
    ///   the incoming message and forwards it to the inner engine, e.g.
    ///     eskf_localizer_->setInitialPose(pose_msg);
    ///     eskf_localizer_->feedObservation(pose_msg / twist_msg);
    ///     eskf_localizer_->feedRelativePose(rel_obs);

private:
    void timerCallback();

    rclcpp::Node::SharedPtr        parent_node_;           ///< host node (use_sim_time, namespace)
    rclcpp::Node::SharedPtr        pnode_;                 ///< private sub-node (parameter prefix)
    std::unique_ptr<ParamListener> param_listener_;        ///< dynamic-parameter listener
    Params                         params_;                ///< latest parameter snapshot
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_; ///< null when publish_tf is false

    std::unique_ptr<EskfLocalizer> eskf_localizer_;        ///< inner ROS-free compute engine

    rclcpp::Logger logger_;

    /// Driver (declared last → destroyed first: the spin thread is stopped and
    /// joined before eskf_localizer_ is torn down, so no tick touches a dead engine).
    rclcpp::TimerBase::SharedPtr               update_timer_;
    std::unique_ptr<finenav::util::NodeThread> pnode_thread_;
};

}  // namespace finenav::localizer
