// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/finenav_localizer.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::localizer {

namespace {
// Fixed internal timer rate [Hz] for the pose-smoothing tick.
inline constexpr double  kInternalTickRateHz = 100.0;
}  // namespace


FineNavLocalizer::Routing FineNavLocalizer::buildRouting(const Params& p) {
    Routing r{};

    const bool has_pose_topic  = !p.pose_topic.empty();
    const bool has_twist_topic = !p.twist_topic.empty();
    const bool has_odom        = !p.odom_topic.empty();

    // ── Pose source ──────────────────────────────────────────────────────
    r.pose = has_pose_topic ? Routing::PoseSrc::Pose
                            : Routing::PoseSrc::None;

    // ── Integration source ───────────────────────────────────────────────
    if (has_twist_topic && has_odom) {
        RCLCPP_WARN(rclcpp::get_logger("finenav_localizer"),
            "FineNavLocalizer: twist_topic ignored for position integration — "
            "odom_topic takes priority.");
    }

    if (has_odom) {
        r.integration = Routing::IntegSrc::RelPose;
    } else if (has_twist_topic) {
        r.integration = Routing::IntegSrc::Twist;
    } else {
        r.integration = Routing::IntegSrc::None;
    }

    // ── Velocity source ──────────────────────────────────────────────────
    if (has_twist_topic && has_odom && p.odom_use_twist) {
        RCLCPP_WARN(rclcpp::get_logger("finenav_localizer"),
            "FineNavLocalizer: odom_use_twist ignored for velocity — "
            "twist_topic takes priority.");
    }

    if (has_twist_topic) {
        r.velocity = Routing::VelSrc::Twist;
    } else if (has_odom && p.odom_use_twist) {
        r.velocity = Routing::VelSrc::Odom;
    } else if (r.pose != Routing::PoseSrc::None) {
        r.velocity = Routing::VelSrc::AutoDiff;
    } else {
        r.velocity = Routing::VelSrc::None;
    }

    return r;
}

FineNavLocalizer::FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                                   rclcpp::NodeOptions pnode_overrides)
    : parent_node_(std::move(parent_node))
{
    pnode_overrides.append_parameter_override(
        "use_sim_time", parent_node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(
        "finenav_localizer", parent_node_->get_name(), pnode_overrides);

    param_listener_ = std::make_unique<ParamListener>(pnode_);
    params_         = param_listener_->get_params();

    // Resolve and validate routing
    routing_ = buildRouting(params_);

    // Create effect sources
    world_anchor_.configure(params_.pose_smoothing_time, kInternalTickRateHz);

    // Subscriptions
    if (!params_.initial_pose_topic.empty()) {
        initial_pose_sub_ = pnode_->create_subscription<PoseMsg>(
            params_.initial_pose_topic, rclcpp::SystemDefaultsQoS(),
            [this](PoseMsg::ConstSharedPtr msg) {
                onInitialPose(*msg);
                initial_pose_sub_.reset();  // one-shot
            });
    }
    if (!params_.pose_topic.empty()) {
        pose_sub_ = pnode_->create_subscription<PoseMsg>(
            params_.pose_topic, rclcpp::SystemDefaultsQoS(),
            [this](PoseMsg::ConstSharedPtr msg) { onPose(*msg); });
    }
    if (!params_.twist_topic.empty()) {
        twist_sub_ = pnode_->create_subscription<TwistMsg>(
            params_.twist_topic, rclcpp::SystemDefaultsQoS(),
            [this](TwistMsg::ConstSharedPtr msg) { onTwist(*msg); });
    }
    if (!params_.odom_topic.empty()) {
        odom_sub_ = pnode_->create_subscription<OdomMsg>(
            params_.odom_topic, rclcpp::SystemDefaultsQoS(),
            [this](OdomMsg::ConstSharedPtr msg) { onOdometry(*msg); });
    }

    // Optional outputs
    if (params_.publish_tf) {
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(pnode_);
    }
    if (params_.publish_odom) {
        nav_state_pub_ = pnode_->create_publisher<nav_msgs::msg::Odometry>(
            "~/nav_state", 10);
    }

    // Timer
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kInternalTickRateHz));

    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period, [this]() { timerCallback(); });

    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);
}


void FineNavLocalizer::onInitialPose(const PoseMsg& msg) {
    onPose(msg);
}

void FineNavLocalizer::onPose(const PoseMsg& msg) {
    const auto& pose = msg.pose.pose;
    const Eigen::Vector3d    obs_p(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond obs_q(pose.orientation.w, pose.orientation.x,
                                    pose.orientation.y, pose.orientation.z);
    const auto stamp = rclcpp::Time(msg.header.stamp);

    {
        std::lock_guard<std::mutex> lk(state_mutex_);

        if (routing_.pose != Routing::PoseSrc::Pose) return;

        // ── Resolve odometry at the observation timestamp ────────────────
        // A delayed pose observation must anchor against the odometry that
        // was current *at the observation instant*, not the latest one.
        // We recover that odometry from the state history:
        //     odom_at_obs = anchor⁻¹ ⊕ state_at_obs
        Eigen::Vector3d    odom_p_at_obs = odom_cache_.position();
        Eigen::Quaterniond odom_q_at_obs = odom_cache_.orientation();
        bool               fallback      = false;

        if (world_anchor_.isReady()) {
            auto state_at_obs = getRobotState(stamp);
            if (!state_at_obs.has_value()) {
                FINENAV_LOG_WARN_THROTTLE(pnode_->get_logger(), 5000,
                    "FineNavLocalizer: pose stamp %.3f s is outside the "
                    "retained state history — falling back to latest odometry.",
                    stamp.seconds());
                fallback = true;
            } else {
                // Back out odometry at T_obs from the stored state
                odom_q_at_obs = world_anchor_.orientation().conjugate()
                                * state_at_obs->orientation;
                odom_p_at_obs = world_anchor_.orientation().conjugate()
                                * (state_at_obs->position - world_anchor_.position());
            }
        }

        // ── Compute the new anchor ───────────────────────────────────────
        //     anchor = obs ⊖ odom_at_obs
        const Eigen::Quaterniond anchor_ori = obs_q * odom_q_at_obs.conjugate();
        const Eigen::Vector3d    anchor_pos = obs_p - anchor_ori * odom_p_at_obs;

        if (params_.pose_smoothing_time <= 0.0) {
            world_anchor_.set(anchor_pos, anchor_ori);
        } else {
            world_anchor_.setTarget(anchor_pos, anchor_ori);
        }

        // ── Compose state from the updated anchor ────────────────────────
        composeState(world_anchor_.position(), world_anchor_.orientation(),
                     odom_cache_.position(), odom_cache_.orientation(),
                     state_.position, state_.orientation);

        // Velocity: auto-diff from consecutive poses
        if (routing_.velocity == Routing::VelSrc::AutoDiff) {
            velocity_.autoDiffFromPose(obs_p, obs_q, stamp, state_);
        }

        initialized_ = true;
    }

    pushState(stamp, state_);
}

void FineNavLocalizer::onTwist(const TwistMsg& msg) {
    const auto& t = msg.twist.twist;
    const Eigen::Vector3d v(t.linear.x, t.linear.y, t.linear.z);
    const Eigen::Vector3d omega(t.angular.x, t.angular.y, t.angular.z);
    const auto stamp = rclcpp::Time(msg.header.stamp);

    {
        std::lock_guard<std::mutex> lk(state_mutex_);

        if (routing_.integration == Routing::IntegSrc::Twist) {
            integrator_.acceptTwist(v, omega, stamp, state_);
        }

        if (routing_.velocity == Routing::VelSrc::Twist) {
            velocity_.setFromTwist(v, omega, state_);
        }

        initialized_ = true;

        if (params_.force_2d_mode) {
            force2D();
        }
        pushState(stamp, state_);
    }
}

void FineNavLocalizer::onOdometry(const OdomMsg& msg) {
    const auto& pose = msg.pose.pose;
    const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
    const Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);

    std::lock_guard<std::mutex> lk(state_mutex_);

    // Always cache the latest odometry
    odom_cache_.apply(p, q);

    // Compose state = anchor ⊕ odometry
    if (world_anchor_.isReady()) {
        composeState(world_anchor_.position(), world_anchor_.orientation(),
                     odom_cache_.position(), odom_cache_.orientation(),
                     state_.position, state_.orientation);
    } else {
        // Bootstrap: no anchor yet — output odometry as-is
        state_.position    = p;
        state_.orientation = q;
    }

    // ── Velocity: odom.twist as explicit velocity ───────────────────────
    if (routing_.velocity == Routing::VelSrc::Odom) {
        const auto& t = msg.twist.twist;
        const Eigen::Vector3d v(t.linear.x, t.linear.y, t.linear.z);
        const Eigen::Vector3d omega(t.angular.x, t.angular.y, t.angular.z);
        velocity_.setFromTwist(v, omega, state_);
    }

    initialized_ = true;

    if (params_.force_2d_mode) {
        force2D();
    }
    pushState(rclcpp::Time(msg.header.stamp), state_);
}

void FineNavLocalizer::timerCallback() {
    // ── Refresh dynamic parameters ───────────────────────────────────────
    if (param_listener_->is_old(params_)) {
        const auto old_smoothing = params_.pose_smoothing_time;
        params_ = param_listener_->get_params();
        if (params_.pose_smoothing_time != old_smoothing) {
            world_anchor_.configure(params_.pose_smoothing_time, kInternalTickRateHz);
        }
    }

    const rclcpp::Time now = pnode_->now();

    {
        std::lock_guard<std::mutex> lk(state_mutex_);

        // ── EMA smoothing tick ──────────────────────────────────────────
        // When smoothing is enabled the WorldAnchor gradually converges
        // the anchor toward each new target.  After each EMA step we
        // recompose the state from the updated anchor + latest odometry.
        if (world_anchor_.isReady()) {
            auto anchor_pos = world_anchor_.position();
            auto anchor_ori = world_anchor_.orientation();
            if (params_.pose_smoothing_time > 0.0 &&
                world_anchor_.tick(anchor_pos, anchor_ori)) {
                if (odom_cache_.hasData()) {
                    composeState(anchor_pos, anchor_ori,
                                 odom_cache_.position(), odom_cache_.orientation(),
                                 state_.position, state_.orientation);
                }
                if (params_.force_2d_mode) {
                    force2D();
                }
                pushState(now, state_);
            }
        }

        publishOutput(now);

        // ── Warning: no absolute pose data ───────────────────────────────
        if (!world_anchor_.hasPoseData()) {
            FINENAV_LOG_WARN_THROTTLE(pnode_->get_logger(), 5000,
                "FineNavLocalizer: no absolute pose source configured or no pose "
                "data received — position will drift without absolute reference.");
        }
    }
}

void FineNavLocalizer::publishOutput(const rclcpp::Time& now) {
    if (tf_br_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now;
        tf.header.frame_id = "map";
        tf.child_frame_id  = "base_link";
        tf.transform.translation.x = state_.position.x();
        tf.transform.translation.y = state_.position.y();
        tf.transform.translation.z = state_.position.z();
        tf.transform.rotation.w = state_.orientation.w();
        tf.transform.rotation.x = state_.orientation.x();
        tf.transform.rotation.y = state_.orientation.y();
        tf.transform.rotation.z = state_.orientation.z();
        tf_br_->sendTransform(tf);
    }

    if (nav_state_pub_) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "map";
        odom.child_frame_id  = "base_link";
        odom.pose.pose.position.x    = state_.position.x();
        odom.pose.pose.position.y    = state_.position.y();
        odom.pose.pose.position.z    = state_.position.z();
        odom.pose.pose.orientation.w = state_.orientation.w();
        odom.pose.pose.orientation.x = state_.orientation.x();
        odom.pose.pose.orientation.y = state_.orientation.y();
        odom.pose.pose.orientation.z = state_.orientation.z();
        odom.twist.twist.linear.x  = state_.linear_velocity.x();
        odom.twist.twist.linear.y  = state_.linear_velocity.y();
        odom.twist.twist.linear.z  = state_.linear_velocity.z();
        odom.twist.twist.angular.x = state_.angular_velocity.x();
        odom.twist.twist.angular.y = state_.angular_velocity.y();
        odom.twist.twist.angular.z = state_.angular_velocity.z();
        nav_state_pub_->publish(odom);
    }
}

void FineNavLocalizer::force2D() {
    state_.position.z() = 0.0;
    const Eigen::Vector3d x_axis = state_.orientation.matrix().col(0);
    const double yaw = std::atan2(x_axis.y(), x_axis.x());
    state_.orientation =
        Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

}  // namespace finenav::localizer
