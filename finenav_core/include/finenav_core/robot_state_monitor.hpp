// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <memory>
#include <shared_mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>
#include <Eigen/Geometry>

#include "finenav_util/node_thread.hpp"

namespace finenav {

// =============================================================================
// RobotState 数据结构
//
// 不含时间戳：时间戳由 RobotStateMonitor 内部以 last_stamp_ 统一维护，
// 通过 getLastStamp() 对外暴露，与状态数据解耦。
// =============================================================================
struct RobotState {
    geometry_msgs::msg::Pose  pose;
    geometry_msgs::msg::Twist velocity;      // linear: world frame;  angular: body frame
    geometry_msgs::msg::Accel acceleration;  // linear / angular: 对速度一阶差分
};

// =============================================================================
// RobotStateMonitor
//
// 设计原则:
//  1. 内部子节点 (pnode_) + NodeThread，与外部节点完全解耦 (对齐 MapServer 模式)
//  2. 读写锁 (shared_mutex):
//       读: shared_lock —— 多线程并发读；返回值拷贝，读完即释放锁，无悬垂引用风险
//       写: unique_lock —— 独占写；100Hz 下 struct 拷贝远比 shared_ptr 分配开销低
//  3. 外部写入口: updateState() / updatePose() / updateVelocity() / updateAcceleration()
//       时间戳统一记录在 last_stamp_，外部调用时取 node_->now()
//  4. TF 跟踪为可选功能，通过 startTFTracking() 启动，stopTFTracking() 终止:
//       内部以 read-modify-write 模式更新 pose（以及可选的数值微分速度/加速度）
//       写回时以 TF transform 的 header.stamp 作为 last_stamp_
// =============================================================================
class RobotStateMonitor {
public:
    /**
     * @brief 构造 RobotStateMonitor (与 MapServer 构造签名对齐)
     * @param node          父节点指针
     * @param monitor_name  监控器名称，用作内部子节点名称
     */
    explicit RobotStateMonitor(rclcpp::Node::SharedPtr node, const std::string& monitor_name);

    RobotStateMonitor(const RobotStateMonitor&) = delete;
    RobotStateMonitor& operator=(const RobotStateMonitor&) = delete;
    ~RobotStateMonitor() = default;

    // =========================================================================
    // 状态读接口
    // =========================================================================

    /**
     * @brief 以值拷贝方式读取当前状态 (shared_lock，支持并发读)
     * @note  100Hz+ 频率下，struct 值拷贝比 shared_ptr 创建/销毁开销更低
     */
    [[nodiscard]] RobotState getCurrentState() const;

    /**
     * @brief 获取最近一次状态更新的时间戳 (shared_lock)
     * @note  TF 跟踪路径下返回 TF transform 的 header.stamp；
     *        外部 update* 路径下返回调用时刻的 node_->now()
     */
    [[nodiscard]] rclcpp::Time getLastStamp() const;

    /**
     * @brief 检查当前状态是否在容忍时间内仍然有效
     * @param tolerance 允许的最大数据过期时长
     * @return true  : (node_->now() - last_stamp_) <= tolerance
     *         false : 状态从未更新过，或已超出容忍时长
     */
    [[nodiscard]] bool isCurrentStateValid(const rclcpp::Duration& tolerance) const;

    // =========================================================================
    // 状态写接口
    // =========================================================================

    /**
     * @brief 原子替换整个状态，时间戳更新为 stamp (默认取 node_->now())
     * @note  TF 跟踪启用时，此接口仍可用于外部补充注入（例如传感器融合的完整状态）
     */
    void updateState(const RobotState& state, const rclcpp::Time& stamp = rclcpp::Time());

    // =========================================================================
    // TF 跟踪生命周期接口
    // =========================================================================

    // 修改点：
    //
    // TF 只是一种特殊的更新源，这个更新源会填入最新的pose 应该仅仅跟踪 map->base_link 的pose （访问周期？）
    // 同时还有其他的更新源，由用户手动调用
    // 当更新周期到来时，如果启用了数值差分选项，进行数值差分（但是数值差分应当是基于 odom->base_link 进行的）；
    /**
     * @brief 启动基于 TF 的状态自动跟踪
     * @param world_frame    世界坐标系名称
     * @param robot_frame    机器人坐标系名称
     * @param update_rate_hz TF 查询频率 (Hz)，默认 50
     * @param numerical_diff 是否通过数值微分估计完整速度/加速度，默认 false
     *                       true:  从 TF 位姿差分推算速度与加速度
     *                       false: 仅更新 pose，速度/加速度保留当前值
     * @note 应在节点启动前调用一次，不应重复调用
     */
    void startTFTracking(const std::string& world_frame,
                         const std::string& robot_frame,
                         double update_rate_hz = 50.0,
                         bool numerical_diff = false);

    /**
     * @brief 永久停止 TF 更新并取消定时器（TF buffer / listener 保留）
     * @note  stop 后不可再重启；若需重新跟踪，请重新调用 startTFTracking
     */
    void stopTFTracking();

private:
    /** @brief 定时器回调: 查询 TF，按配置计算速度/加速度，以 read-modify-write 写回状态 */
    void runTFUpdateCycle();

    // ---- 节点与线程 (对齐 MapServer) ----
    rclcpp::Node::SharedPtr      node_;
    rclcpp::Node::SharedPtr      pnode_;
    std::unique_ptr<NodeThread>  pnode_thread_;

    // ---- 读写锁保护的状态与时间戳 ----
    mutable std::shared_mutex  state_mutex_;
    RobotState                 current_state_;
    rclcpp::Time               last_stamp_{0, 0, RCL_ROS_TIME};

    // ---- TF 配置 (startTFTracking 中 set-once，运行期不变) ----
    std::string  world_frame_;
    std::string  robot_frame_;
    bool         numerical_diff_enabled_{false};

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr                update_timer_;

    // ---- 数值微分缓存 (仅在 pnode_ 单线程执行器的定时器回调中访问，无需加锁) ----
    bool                       has_prev_{false};
    rclcpp::Time               prev_stamp_{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Pose   prev_pose_{};
    geometry_msgs::msg::Twist  prev_velocity_{};
};

}  // namespace finenav

