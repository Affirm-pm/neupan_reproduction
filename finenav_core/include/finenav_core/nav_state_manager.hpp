// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>

#include "finenav_msgs/msg/nav_status.hpp"
#include "finenav_localizer/finenav_localizer.hpp"

namespace finenav {

/**
 * @brief NavStateManager — 系统唯一的导航状态管理实体。
 *
 * 宿主：Navigator 组件，通过 shared_ptr 与 ControlLayer 在进程内共享。
 *
 * 职责：
 *  1. 维护导航 FSM
 *  2. 通过内部定时器轮询 FineNavLocalizer，自主驱动状态转移
 *  3. 累积本次导航里程
 *  4. 向 /finenav/nav_status 发布 NavStatus 供外部监控
 *
 * FSM 转移：
 *  startEpisode(goal)  → NAVIGATING          （Navigator 在 BT 开始时调用）
 *  timer: dist < tol   → GOAL_REACHED        （自主检测）
 *  timer: 位移超时      → STUCK               （自主检测）
 *  timer: 定位异常      → LOCALIZATION_ERROR  （自主检测）
 *  setAborted()        → ABORTED             （Navigator 在 BT 整体 FAILURE 时调用）
 *  reset()             → IDLE               （Navigator 在 cancel / preempt 时调用）
 *
 * 线程安全：timerCallback() 与所有公开接口均受 mutex_ 保护。
 */
class NavStateManager {
public:
    enum class State : uint8_t {
        IDLE               = finenav_msgs::msg::NavStatus::IDLE,
        NAVIGATING         = finenav_msgs::msg::NavStatus::NAVIGATING,
        GOAL_REACHED       = finenav_msgs::msg::NavStatus::GOAL_REACHED,
        STUCK              = finenav_msgs::msg::NavStatus::STUCK,
        ABORTED            = finenav_msgs::msg::NavStatus::ABORTED,
        LOCALIZATION_ERROR = finenav_msgs::msg::NavStatus::LOCALIZATION_ERROR,
    };

    struct Config {
        double goal_tolerance               = 0.25;
        double stuck_dist_threshold         = 0.05;
        double stuck_timeout_s              = 5.0;
        double localization_cov_threshold   = 5.0;
        double localization_stale_timeout_s = 1.0;
        double monitor_rate_hz              = 10.0;
        Config() = default;
    };

    explicit NavStateManager(rclcpp::Node::SharedPtr node,
                             FineNavLocalizer&        localizer);

    explicit NavStateManager(rclcpp::Node::SharedPtr node,
                             FineNavLocalizer&        localizer,
                             Config                   cfg);

    // ── Episode 边界接口（由 Navigator 调用）──────────────────────────────

    /** 开始新 episode：重置所有状态量，进入 NAVIGATING。 */
    void startEpisode(const geometry_msgs::msg::Pose& goal_pose) {
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

    /** BT 整体 FAILURE 时由 Navigator 调用，进入 ABORTED。 */
    void setAborted() {
        std::lock_guard<std::mutex> lk(mutex_);
        if (state_ == State::NAVIGATING) {
            state_ = State::ABORTED;
            RCLCPP_WARN(node_->get_logger(), "[NavStateManager] setAborted → ABORTED.");
        }
    }

    /** cancel / preempt 时由 Navigator 调用，进入 IDLE。 */
    void reset() {
        std::lock_guard<std::mutex> lk(mutex_);
        state_             = State::IDLE;
        distance_traveled_ = 0.0;
        has_last_pos_      = false;
        stuck_ref_set_     = false;
        RCLCPP_INFO(node_->get_logger(), "[NavStateManager] reset → IDLE.");
    }

    // ── 查询接口（由 ControlLayer TrackingPolicy 轮询）────────────────────

    State getState() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return state_;
    }

    /** 返回 true 表示 TrackingPolicy 应当停止规划循环。 */
    bool shouldStop() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return state_ == State::GOAL_REACHED       ||
               state_ == State::STUCK              ||
               state_ == State::ABORTED            ||
               state_ == State::LOCALIZATION_ERROR;
    }

    double getDistanceTraveled() const {
        std::lock_guard<std::mutex> lk(mutex_);
        return distance_traveled_;
    }

private:
    // ── 内部定时器回调 ─────────────────────────────────────────────────────

    void timerCallback() {
        // 在 mutex_ 外读取 localizer（localizer 有自己的读写锁）
        const auto robot_state = localizer_.getState();
        const auto covariance  = localizer_.getCovariance();
        const auto last_update = localizer_.getLastUpdateTime();

        std::lock_guard<std::mutex> lk(mutex_);

        if (state_ != State::NAVIGATING) {
            publishLocked(robot_state);
            return;
        }

        const Eigen::Vector2d cur_pos(robot_state.p.x(), robot_state.p.y());

        // ── 1. 里程累积 ─────────────────────────────────────────────────
        if (has_last_pos_) {
            distance_traveled_ += (cur_pos - last_pos_).norm();
        }
        last_pos_     = cur_pos;
        has_last_pos_ = true;

        // ── 2. 定位异常检测 → LOCALIZATION_ERROR ────────────────────────
        const double pos_cov_trace = covariance.template block<3, 3>(0, 0).trace();
        if (pos_cov_trace > cfg_.localization_cov_threshold) {
            state_ = State::LOCALIZATION_ERROR;
            RCLCPP_ERROR(node_->get_logger(),
                "[NavStateManager] LOCALIZATION_ERROR: cov_trace=%.3f > %.3f",
                pos_cov_trace, cfg_.localization_cov_threshold);
            publishLocked(robot_state);
            return;
        }
        const double stale_s = (node_->now() - last_update).seconds();
        if (stale_s > cfg_.localization_stale_timeout_s) {
            state_ = State::LOCALIZATION_ERROR;
            RCLCPP_ERROR(node_->get_logger(),
                "[NavStateManager] LOCALIZATION_ERROR: localizer stale %.2f s", stale_s);
            publishLocked(robot_state);
            return;
        }

        // ── 3. 目标到达检测 → GOAL_REACHED ──────────────────────────────
        const Eigen::Vector2d goal_pos(goal_pose_.position.x, goal_pose_.position.y);
        const double dist_to_goal = (cur_pos - goal_pos).norm();
        if (dist_to_goal < cfg_.goal_tolerance) {
            state_ = State::GOAL_REACHED;
            RCLCPP_INFO(node_->get_logger(),
                "[NavStateManager] GOAL_REACHED. dist=%.3f m, traveled=%.2f m",
                dist_to_goal, distance_traveled_);
            publishLocked(robot_state);
            return;
        }

        // ── 4. Stuck 检测 → STUCK ────────────────────────────────────────
        if (!stuck_ref_set_) {
            stuck_ref_pos_      = cur_pos;
            stuck_ref_set_      = true;
            stuck_window_start_ = node_->now();
        } else {
            const double moved = (cur_pos - stuck_ref_pos_).norm();
            if (moved > cfg_.stuck_dist_threshold) {
                // 有足够移动，重置 stuck 检测窗口
                stuck_ref_pos_      = cur_pos;
                stuck_window_start_ = node_->now();
            } else {
                const double elapsed = (node_->now() - stuck_window_start_).seconds();
                if (elapsed > cfg_.stuck_timeout_s) {
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

    /** 调用方已持有 mutex_，直接构造并发布 NavStatus。 */
    void publishLocked(const NavStateD& rs) {
        finenav_msgs::msg::NavStatus msg;
        msg.header.stamp    = node_->now();
        msg.header.frame_id = "map";
        msg.state           = static_cast<uint8_t>(state_);
        msg.distance_traveled   = distance_traveled_;
        // 车体系线速度：v_body = R^{-1} * v_world
        const Eigen::Vector3d v_body = rs.R.inverse() * rs.v;
        msg.current_linear_vel  = v_body.x();
        msg.current_angular_vel = rs.omega.z();
        msg.goal_pose           = goal_pose_;
        status_pub_->publish(msg);
    }

    // ── 数据成员 ──────────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr  node_;
    FineNavLocalizer&         localizer_;
    Config                    cfg_;
    mutable std::mutex        mutex_;

    State                     state_{State::IDLE};
    geometry_msgs::msg::Pose  goal_pose_;

    // 里程
    bool             has_last_pos_{false};
    Eigen::Vector2d  last_pos_{0.0, 0.0};
    double           distance_traveled_{0.0};

    // Stuck 检测
    bool             stuck_ref_set_{false};
    Eigen::Vector2d  stuck_ref_pos_{0.0, 0.0};
    rclcpp::Time     stuck_window_start_;

    rclcpp::TimerBase::SharedPtr                               monitor_timer_;
    rclcpp::Publisher<finenav_msgs::msg::NavStatus>::SharedPtr status_pub_;
};

} // namespace finenav

// Out-of-class definitions to work around GCC bug #88165.
inline finenav::NavStateManager::NavStateManager(
    rclcpp::Node::SharedPtr node,
    finenav::FineNavLocalizer& localizer)
    : NavStateManager(std::move(node), localizer, Config{}) {}

inline finenav::NavStateManager::NavStateManager(
    rclcpp::Node::SharedPtr node,
    finenav::FineNavLocalizer& localizer,
    finenav::NavStateManager::Config cfg)
    : node_(std::move(node)), localizer_(localizer), cfg_(cfg)
{
    status_pub_ = node_->create_publisher<finenav_msgs::msg::NavStatus>(
        "/finenav/nav_status", rclcpp::SystemDefaultsQoS());

    monitor_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / cfg_.monitor_rate_hz),
        std::bind(&NavStateManager::timerCallback, this));

    RCLCPP_INFO(node_->get_logger(),
        "[NavStateManager] initialized (monitor %.1f Hz).", cfg_.monitor_rate_hz);
}

