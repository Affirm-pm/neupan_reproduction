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
#include "finenav_navigator/nav_state_manager_params.hpp"

namespace finenav {

/**
 * @brief NavStateManager — 系统唯一的导航状态管理实体。
 *
 * 参数通过 generate_parameter_library 从 nav_state_manager.yaml 加载。
 * 构造时传入宿主节点，内部创建 ParamListener 读取参数。
 * Episode 边界接口（startEpisode / setAborted / reset）由 Navigator 调用。
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

    using Params        = nav_state_manager::Params;
    using ParamListener = nav_state_manager::ParamListener;

    /**
     * @param node      宿主节点
     * @param localizer 依赖注入的定位器引用
     */
    explicit NavStateManager(rclcpp::Node::SharedPtr node,
                             FineNavLocalizer&        localizer);

    // ── Episode 边界接口（由 Navigator 调用）──────────────────────────────

    /** 开始新 episode：重置所有状态量，进入 NAVIGATING。 */
    void startEpisode(const geometry_msgs::msg::Pose& goal_pose);

    /** BT 整体 FAILURE 时由 Navigator 调用，进入 ABORTED。 */
    void setAborted();

    /** cancel / preempt 时由 Navigator 调用，进入 IDLE。 */
    void reset();

    // ── 查询接口 ──────────────────────────────────────────────────────────

    State getState() const;

    /** 返回 true 表示 TrackingPolicy 应当停止规划循环。 */
    bool shouldStop() const;

    double getDistanceTraveled() const;

private:
    void timerCallback();
    void publishLocked(const NavStateD& rs);

    // ── 数据成员 ──────────────────────────────────────────────────────────
    rclcpp::Node::SharedPtr     node_;
    FineNavLocalizer&           localizer_;
    std::unique_ptr<ParamListener> param_listener_;
    Params                      params_;
    mutable std::mutex          mutex_;

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

