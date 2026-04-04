// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_core/robot_state_monitor.hpp"

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace finenav {

// =============================================================================
// 构造
// =============================================================================

RobotStateMonitor::RobotStateMonitor(rclcpp::Node::SharedPtr node, const std::string& monitor_name)
    : node_(node)
{
    pnode_ = rclcpp::Node::make_shared(monitor_name, node_->get_name(), rclcpp::NodeOptions());
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    RCLCPP_INFO(pnode_->get_logger(), "RobotStateMonitor '%s' initialized", monitor_name.c_str());
}

// =============================================================================
// 状态读接口
// =============================================================================

RobotState RobotStateMonitor::getCurrentState() const {
    std::shared_lock lock(state_mutex_);
    return current_state_;  // 值拷贝；解锁后调用方独立持有，无悬垂风险
}

rclcpp::Time RobotStateMonitor::getLastStamp() const {
    std::shared_lock lock(state_mutex_);
    return last_stamp_;
}

bool RobotStateMonitor::isCurrentStateValid(const rclcpp::Duration& tolerance) const {
    std::shared_lock lock(state_mutex_);
    // 从未更新过（last_stamp_ 仍为零）视为无效
    if (last_stamp_.nanoseconds() == 0) {
        return false;
    }
    return (node_->now() - last_stamp_) <= tolerance;
}

// =============================================================================
// 状态写接口
// =============================================================================

void RobotStateMonitor::updateState(const RobotState& state, const rclcpp::Time& stamp) {
    std::unique_lock lock(state_mutex_);
    auto effective_stamp = stamp.nanoseconds() == 0 ? node_->now() : stamp;

    if (last_stamp_.nanoseconds() > 0 && effective_stamp < last_stamp_) {
        RCLCPP_WARN(node_->get_logger(), "[RobotStateMonitor] : incoming stamp (%.6f s) is older than "
            "last_stamp_ (%.6f s), update rejected.", effective_stamp.seconds(), last_stamp_.seconds()); // TODO: fmt
        return;
    }
    current_state_ = state;
    last_stamp_    = effective_stamp;
}

// =============================================================================
// TF 跟踪生命周期
// =============================================================================

void RobotStateMonitor::startTFTracking(const std::string& world_frame,
                                         const std::string& robot_frame,
                                         double update_rate_hz,
                                         bool numerical_diff)
{
    world_frame_            = world_frame;
    robot_frame_            = robot_frame;
    numerical_diff_enabled_ = numerical_diff;

    // 懒创建 TF 基础设施（TransformListener 必须在所属节点的 executor 中运行）
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(pnode_->get_clock()); // TODO: 为什么这里是 pnode 时钟？
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, pnode_);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz));

    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period,
        [this]() { this->runTFUpdateCycle(); });

    RCLCPP_INFO(pnode_->get_logger(),
        "TF tracking started: [%s] -> [%s] @ %.1f Hz%s",
        world_frame.c_str(), robot_frame.c_str(), update_rate_hz,
        numerical_diff ? " +numerical_diff" : "");
}

void RobotStateMonitor::stopTFTracking() {
    if (update_timer_) {
        update_timer_->cancel();
        update_timer_.reset();
        RCLCPP_INFO(pnode_->get_logger(), "TF tracking stopped");
    }
}

// =============================================================================
// 定时器回调（pnode_ SingleThreadedExecutor 保证串行，微分缓存无需加锁）
// =============================================================================

void RobotStateMonitor::runTFUpdateCycle() {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(world_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG(pnode_->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    const rclcpp::Time stamp(transform.header.stamp);

    // --- read-modify-write: 读取当前状态副本（shared_lock），仅覆盖 TF 负责的字段 ---
    RobotState next;
    {
        std::shared_lock lock(state_mutex_);
        next = current_state_;
    }

    next.pose.position.x  = transform.transform.translation.x;
    next.pose.position.y  = transform.transform.translation.y;
    next.pose.position.z  = transform.transform.translation.z;
    next.pose.orientation = transform.transform.rotation;

    // ---- 数值微分: 完整速度  ---- // TODO: 正确性有待商榷
    if (numerical_diff_enabled_ && has_prev_) {
        const double dt = (stamp - prev_stamp_).seconds();
        // 过滤无效 dt（时钟跳变 / 首帧 / 长时间中断后恢复）
        if (dt > 1e-6 && dt < 1.0) {

            // 线速度: 位置一阶差分 (world frame)
            const auto& p  = next.pose.position;
            const auto& pp = prev_pose_.position;
            next.velocity.linear.x = (p.x - pp.x) / dt;
            next.velocity.linear.y = (p.y - pp.y) / dt;
            next.velocity.linear.z = (p.z - pp.z) / dt;

            // 角速度: 四元数有限差分 → ω_body = 2 * q_conj * q_dot (body frame)
            Eigen::Quaterniond q_prev(prev_pose_.orientation.w,
                                      prev_pose_.orientation.x,
                                      prev_pose_.orientation.y,
                                      prev_pose_.orientation.z);
            Eigen::Quaterniond q_next(next.pose.orientation.w,
                                      next.pose.orientation.x,
                                      next.pose.orientation.y,
                                      next.pose.orientation.z);
            q_prev.normalize();
            q_next.normalize();

            Eigen::Quaterniond q_dot;
            q_dot.w() = (q_next.w() - q_prev.w()) / dt;
            q_dot.x() = (q_next.x() - q_prev.x()) / dt;
            q_dot.y() = (q_next.y() - q_prev.y()) / dt;
            q_dot.z() = (q_next.z() - q_prev.z()) / dt;

            const Eigen::Quaterniond omega_q = q_prev.conjugate() * q_dot;
            next.velocity.angular.x = 2.0 * omega_q.x();
            next.velocity.angular.y = 2.0 * omega_q.y();
            next.velocity.angular.z = 2.0 * omega_q.z();
        }
    }

    // 更新微分基准（仅本线程访问，无需加锁）
    prev_pose_     = next.pose;
    prev_velocity_ = next.velocity;
    prev_stamp_    = stamp;
    has_prev_      = true;

    // 写回（unique_lock），以 TF transform 的 stamp 作为 last_stamp_
    {
        std::unique_lock lock(state_mutex_);
        current_state_ = std::move(next);
        last_stamp_    = stamp;
    }
}

}  // namespace finenav
