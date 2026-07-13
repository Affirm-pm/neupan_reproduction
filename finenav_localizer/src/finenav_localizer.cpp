// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/finenav_localizer.hpp"

#include <chrono>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::localizer {

namespace {
/// Engine state (NavStateD, v in body frame) → canonical core RobotState.
finenav::core::RobotState toRobotState(const NavStateD & s)
{
    return finenav::core::RobotState{s.p, s.R.unit_quaternion(), s.v, s.omega};
}
}  // namespace

// ============================================================================
//  Constructor
// ============================================================================
FineNavLocalizer::FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                                   rclcpp::NodeOptions pnode_overrides)
    : parent_node_(std::move(parent_node))
    , logger_(rclcpp::get_logger("finenav.FineNavLocalizer"))
{
    // Private sub-node (parameter prefix from the yaml top-level key).
    pnode_overrides.append_parameter_override(
        "use_sim_time",
        parent_node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(
        "finenav_localizer", parent_node_->get_name(), pnode_overrides);

    // Parameters → inner engine Config.
    param_listener_ = std::make_unique<ParamListener>(pnode_);
    params_         = param_listener_->get_params();
    eskf_localizer_ = std::make_unique<EskfLocalizer>(configFromParams(params_));

    // Size the inherited RobotStateProvider history window from the same param.
    setHistoryDuration(params_.history_duration_sec);

    // TF broadcaster (only if publish_tf is enabled).
    if (params_.publish_tf) {
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(pnode_);
    }

    // Update timer, then spin thread (last, so all entities are registered).
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / params_.update_rate_hz));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period, [this]() { timerCallback(); });
    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);

    FINENAV_LOG_INFO(logger_,
        "[FineNavLocalizer] ROS wrapper ready. rate={:.1f} Hz  publish_tf={}",
        params_.update_rate_hz, params_.publish_tf ? "true" : "false");
}

// ============================================================================
//  Params → EskfLocalizer::Config
// ============================================================================
EskfLocalizer::Config FineNavLocalizer::configFromParams(const Params & p) const
{
    EskfLocalizer::Config c;
    c.passthrough              = p.passthrough;
    c.update_rate_hz           = p.update_rate_hz;
    c.max_delay_sec            = p.max_delay_sec;
    c.pose_smoothing_steps     = p.pose_smoothing_steps;
    c.twist_smoothing_steps    = p.twist_smoothing_steps;
    c.rel_pose_smoothing_steps = p.rel_pose_smoothing_steps;
    c.pose_gate_dist           = p.pose_gate_dist;
    c.twist_gate_dist          = p.twist_gate_dist;
    c.rel_pose_gate_dist       = p.rel_pose_gate_dist;
    c.max_dt_sec               = p.max_dt_sec;
    c.max_obs_queue_size       = p.max_obs_queue_size;
    c.max_rel_pose_queue_size  = p.max_rel_pose_queue_size;
    c.force_2d_mode            = p.force_2d_mode;
    c.vel_noise_std            = p.model.vel_noise_std;
    c.omega_noise_std          = p.model.omega_noise_std;
    return c;
}

// ============================================================================
//  timerCallback — refresh params → step engine → TF → overrun check
// ============================================================================
void FineNavLocalizer::timerCallback()
{
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        eskf_localizer_->setConfig(configFromParams(params_));
    }

    const rclcpp::Time tick_time = pnode_->now();
    eskf_localizer_->step(tick_time);

    // Feed the latest state into the inherited RobotStateProvider history.
    if (eskf_localizer_->isInitialized()) {
        pushState(tick_time, toRobotState(eskf_localizer_->getState()));
    }

    // ── Broadcast TF: map → base_link (raw world-frame state) ────────────────
    if (tf_br_ && eskf_localizer_->isInitialized()) {
        const NavStateD s = eskf_localizer_->getRawState();

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = tick_time;
        tf.header.frame_id = "map";
        tf.child_frame_id  = "base_link";
        tf.transform.translation.x = s.p.x();
        tf.transform.translation.y = s.p.y();
        tf.transform.translation.z = s.p.z();
        const Eigen::Quaterniond q(s.R.matrix());
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf_br_->sendTransform(tf);
    }

    // ── Overrun detection (real-time concern; lives in the ROS layer) ────────
    const double elapsed_sec = (pnode_->now() - tick_time).seconds();
    const double period_sec  = 1.0 / params_.update_rate_hz;
    FINENAV_LOG_INFO_THROTTLE(logger_, 2000,
        "[FineNavLocalizer] tick complete  elapsed={:.1f} ms  period={:.1f} ms",
        elapsed_sec * 1e3, period_sec * 1e3);
    if (elapsed_sec > period_sec) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "[FineNavLocalizer] Timer overrun: callback took {:.1f} ms but period is {:.1f} ms "
            "({:.0f}% over). Consider reducing update_rate_hz or max_delay_step.",
            elapsed_sec * 1e3, period_sec * 1e3, (elapsed_sec / period_sec - 1.0) * 100.0);
    }
}

// ============================================================================
//  Output API — delegate to the inner engine
// ============================================================================
NavStateD FineNavLocalizer::getState() const
{
    return eskf_localizer_->getState();
}

FineNavLocalizer::PMat FineNavLocalizer::getCovariance() const
{
    return eskf_localizer_->getCovariance();
}

rclcpp::Time FineNavLocalizer::getLastUpdateTime() const
{
    return eskf_localizer_->getLastUpdateTime();
}

}  // namespace finenav::localizer
