// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/nav_state_manager.hpp"

namespace finenav {

NavStateManager::NavStateManager(rclcpp::Node::SharedPtr node,
                                 FineNavLocalizer&        localizer)
    : node_(std::move(node)), localizer_(localizer)
{
    param_listener_ = std::make_unique<ParamListener>(node_);
    params_ = param_listener_->get_params();

    status_pub_ = node_->create_publisher<finenav_msgs::msg::NavStatus>(
        "/finenav/nav_status", rclcpp::SystemDefaultsQoS());

    monitor_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / params_.monitor_rate_hz),
        std::bind(&NavStateManager::timerCallback, this));

    RCLCPP_INFO(node_->get_logger(),
        "[NavStateManager] initialized (monitor %.1f Hz, goal_tol=%.2f m).",
        params_.monitor_rate_hz, params_.goal_tolerance);
}

// ── Episode 边界接口 ───────────────────────────────────────────────────────────

void NavStateManager::startEpisode(const geometry_msgs::msg::Pose& goal_pose)
{
    std::lock_guard<std::mutex> lk(mutex_);
    goal_pose_         = goal_pose;
    state_             = State::NAVIGATING;
    distance_traveled_ = 0.0;
    has_last_pos_      = false;
    stuck_ref_set_     = false;
    RCLCPP_INFO(node_->get_logger(),
        "[NavStateManager] startEpisode → NAVIGATING. Goal=(%.2f, %.2f)",
        goal_pose.position.x, goal_pose.position.y);
}

void NavStateManager::setAborted()
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (state_ == State::NAVIGATING) {
        state_ = State::ABORTED;
        RCLCPP_WARN(node_->get_logger(), "[NavStateManager] setAborted → ABORTED.");
    }
}

void NavStateManager::reset()
{
    std::lock_guard<std::mutex> lk(mutex_);
    state_             = State::IDLE;
    distance_traveled_ = 0.0;
    has_last_pos_      = false;
    stuck_ref_set_     = false;
    RCLCPP_INFO(node_->get_logger(), "[NavStateManager] reset → IDLE.");
}

// ── 查询接口 ──────────────────────────────────────────────────────────────────

NavStateManager::State NavStateManager::getState() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    return state_;
}

bool NavStateManager::shouldStop() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    return state_ == State::GOAL_REACHED       ||
           state_ == State::STUCK              ||
           state_ == State::ABORTED            ||
           state_ == State::LOCALIZATION_ERROR;
}

double NavStateManager::getDistanceTraveled() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    return distance_traveled_;
}

// ── 内部定时器回调 ────────────────────────────────────────────────────────────

void NavStateManager::timerCallback()
{
    const auto robot_state = localizer_.getState();
    const auto covariance  = localizer_.getCovariance();
    const auto last_update = localizer_.getLastUpdateTime();

    std::lock_guard<std::mutex> lk(mutex_);

    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
    }

    if (state_ != State::NAVIGATING) {
        publishLocked(robot_state);
        return;
    }

    const Eigen::Vector2d cur_pos(robot_state.p.x(), robot_state.p.y());

    // ── 1. 里程累积 ──────────────────────────────────────────────────────────
    if (has_last_pos_) {
        distance_traveled_ += (cur_pos - last_pos_).norm();
    }
    last_pos_     = cur_pos;
    has_last_pos_ = true;

    // ── 2. 定位异常检测 ──────────────────────────────────────────────────────
    const double pos_cov_trace = covariance.template block<3, 3>(0, 0).trace();
    if (pos_cov_trace > params_.localization_cov_threshold) {
        state_ = State::LOCALIZATION_ERROR;
        RCLCPP_ERROR(node_->get_logger(),
            "[NavStateManager] LOCALIZATION_ERROR: cov_trace=%.3f > %.3f",
            pos_cov_trace, params_.localization_cov_threshold);
        publishLocked(robot_state);
        return;
    }
    const double stale_s = (node_->now() - last_update).seconds();
    if (stale_s > params_.localization_stale_timeout_s) {
        state_ = State::LOCALIZATION_ERROR;
        RCLCPP_ERROR(node_->get_logger(),
            "[NavStateManager] LOCALIZATION_ERROR: localizer stale %.2f s", stale_s);
        publishLocked(robot_state);
        return;
    }

    // ── 3. 目标到达检测 ──────────────────────────────────────────────────────
    const Eigen::Vector2d goal_pos(goal_pose_.position.x, goal_pose_.position.y);
    const double dist_to_goal = (cur_pos - goal_pos).norm();
    if (dist_to_goal < params_.goal_tolerance) {
        state_ = State::GOAL_REACHED;
        RCLCPP_INFO(node_->get_logger(),
            "[NavStateManager] GOAL_REACHED. dist=%.3f m, traveled=%.2f m",
            dist_to_goal, distance_traveled_);
        publishLocked(robot_state);
        return;
    }

    // ── 4. Stuck 检测 ────────────────────────────────────────────────────────
    if (!stuck_ref_set_) {
        stuck_ref_pos_      = cur_pos;
        stuck_ref_set_      = true;
        stuck_window_start_ = node_->now();
    } else {
        const double moved = (cur_pos - stuck_ref_pos_).norm();
        if (moved > params_.stuck_dist_threshold) {
            stuck_ref_pos_      = cur_pos;
            stuck_window_start_ = node_->now();
        } else {
            const double elapsed = (node_->now() - stuck_window_start_).seconds();
            if (elapsed > params_.stuck_timeout_s) {
                state_ = State::STUCK;
                RCLCPP_WARN(node_->get_logger(),
                    "[NavStateManager] STUCK: moved %.4f m in %.1f s.",
                    moved, elapsed);
                publishLocked(robot_state);
                return;
            }
        }
    }

    publishLocked(robot_state);
}

void NavStateManager::publishLocked(const NavStateD& rs)
{
    finenav_msgs::msg::NavStatus msg;
    msg.header.stamp    = node_->now();
    msg.header.frame_id = "map";
    msg.state           = static_cast<uint8_t>(state_);
    msg.distance_traveled   = distance_traveled_;
    const Eigen::Vector3d v_body = rs.R.inverse() * rs.v;
    msg.current_linear_vel  = v_body.x();
    msg.current_angular_vel = rs.omega.z();
    msg.goal_pose           = goal_pose_;
    status_pub_->publish(msg);
}

} // namespace finenav

