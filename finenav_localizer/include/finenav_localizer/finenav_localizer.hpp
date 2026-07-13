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

// ============================================================================
//  FineNavLocalizer  —  ROS wrapper around the EskfLocalizer engine
//
//  This is the outer half of the localizer.  It owns the ROS runtime — a private
//  sub-node, a spin thread, a periodic timer, the parameter listener and the TF
//  broadcaster — and drives the inner EskfLocalizer (the ROS-free C++ engine):
//    • maps the generated Params → EskfLocalizer::Config,
//    • each timer tick: refresh params → eskf_localizer_->step(pnode_->now())
//      → broadcast map→base_link TF → overrun check,
//    • is a RobotStateProvider: each tick it pushes the engine's latest RobotState
//      into the inherited history; getRobotState()/(t) are served by the base.
//
//  Input side (subscriptions) is intentionally NOT implemented here yet — see the
//  TODO below.  For direct C++ use (benchmarks / tests / offline eval), construct
//  and drive EskfLocalizer directly.
// ============================================================================
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

    // ── Output API (delegates to the inner EskfLocalizer) ────────────────────
    /// Latest state, `v` in the body frame.  Thread-safe.
    [[nodiscard]] NavStateD    getState()          const;
    [[nodiscard]] PMat         getCovariance()     const;
    [[nodiscard]] rclcpp::Time getLastUpdateTime() const;

    //  getRobotState() / getRobotState(t) are inherited from RobotStateProvider
    //  (history-backed); this wrapper feeds the history via pushState() each tick.

    // ── TODO: ROS message-subscription input side (future work) ──────────────
    //  This wrapper will own the localizer's ROS inputs: subscriptions to the
    //  initial-pose, pose, twist and odometry topics.  Each callback converts the
    //  incoming message and forwards it to the inner engine, e.g.
    //      eskf_localizer_->setInitialPose(pose_msg);
    //      eskf_localizer_->feedObservation(pose_msg / twist_msg);
    //      eskf_localizer_->feedRelativePose(rel_obs);
    //  Not implemented yet — add create_subscription<...>() here and wire the
    //  callbacks to eskf_localizer_.

private:
    void timerCallback();
    [[nodiscard]] EskfLocalizer::Config configFromParams(const Params & p) const;

    // ── ROS infrastructure ───────────────────────────────────────────────────
    rclcpp::Node::SharedPtr        parent_node_;
    rclcpp::Node::SharedPtr        pnode_;
    std::unique_ptr<ParamListener> param_listener_;
    Params                         params_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;  // null when publish_tf is false

    // ── Inner compute engine ─────────────────────────────────────────────────
    std::unique_ptr<EskfLocalizer> eskf_localizer_;

    rclcpp::Logger logger_;

    // ── Driver (declared last → destroyed first: the spin thread is stopped and
    //     joined before eskf_localizer_ is torn down, so no tick touches a dead
    //     engine) ───────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr               update_timer_;
    std::unique_ptr<finenav::util::NodeThread> pnode_thread_;
};

}  // namespace finenav::localizer
