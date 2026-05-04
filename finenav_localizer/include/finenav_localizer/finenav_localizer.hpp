// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "eskf.hpp"
#include "finenav_localizer/filter_model.hpp"
#include "finenav_localizer/observation_queue.hpp"
#include "finenav_util/node_thread.hpp"
#include "finenav_localizer/localizer_params.hpp"

namespace finenav {

// ============================================================================
//  FineNavLocalizer  —  timer-driven ESKF localizer with time-delay compensation
//
//  参数通过 generate_parameter_library 从 finenav_localizer.yaml 加载，
//  与框架其他组件（Navigator、NavStateManager）保持一致的配置风格。
//
//  Internal threading model
//  ─────────────────────────────────────────────────────────────────────────
//  A private sub-node (pnode_) runs inside a dedicated NodeThread.
//  timerCallback() is the sole writer of eskf_.
//
//  setInitialPose() may be called from any external thread; it uses
//  filter_mutex_ to serialise access with timerCallback().
//
//  feedObservation() pushes into pose_queue_ / twist_queue_, which are each
//  mutex-protected internally; the call never contends with timerCallback().
//
//  getState() / getCovariance() / getLastUpdateTime() read from an output
//  cache protected by cached_state_mutex_ (shared_mutex).
// ============================================================================
class FineNavLocalizer {
public:
    using PMat     = NavigationModelD::PMat;
    using Vec3d    = NavigationModelD::Vec3T;
    using SO3d     = NavigationModelD::SO3T;
    using PoseMsg  = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;

    /// Convenience aliases for the generated parameter types.
    using Params        = finenav_localizer::Params;
    using ParamListener = finenav_localizer::ParamListener;

    /**
     * @brief 构造 FineNavLocalizer。
     * @param parent_node     宿主节点（用于获取 use_sim_time 及命名空间继承）。
     *                        内部会创建名为 "finenav_localizer" 的私有子节点来读取参数。
     * @param pnode_overrides 可选的参数覆盖（测试/eval场景用），通过
     *                        rclcpp::NodeOptions::append_parameter_override() 设置。
     */
    explicit FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                              rclcpp::NodeOptions pnode_overrides = rclcpp::NodeOptions{});

    FineNavLocalizer(const FineNavLocalizer &)             = delete;
    FineNavLocalizer & operator=(const FineNavLocalizer &) = delete;
    ~FineNavLocalizer() = default;

    /**
     * @brief Reset the filter to the given pose.  Clears all delay-slot history.
     * @note  Thread-safe.
     */
    void setInitialPose(const PoseMsg & msg);

    /**
     * @brief Enqueue a pose or twist observation for the next timer tick.
     * @note  Thread-safe.
     */
    void feedObservation(const PoseMsg  & msg);
    void feedObservation(const TwistMsg & msg);

    /**
     * @brief Read-only access to the latest filter output.
     * @note  Thread-safe.
     */
    NavStateD    getState()          const;
    PMat         getCovariance()     const;
    rclcpp::Time getLastUpdateTime() const;

private:

    // ── Timer callback ───────────────────────────────────────────────────────
    void timerCallback();

    // ── Observation update (called inside timerCallback, single-threaded) ────
    void applyUpdate(const PoseMsg  & msg, int delay_step);
    void applyUpdate(const TwistMsg & msg, int delay_step);

    // ── Message extraction helpers ───────────────────────────────────────────
    static std::tuple<Vec3d, SO3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const PoseMsg & msg);

    static std::tuple<Vec3d, Vec3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const TwistMsg & msg);

    // ── ROS infrastructure ───────────────────────────────────────────────────
    rclcpp::Node::SharedPtr                        parent_node_;
    rclcpp::Node::SharedPtr                        pnode_;
    std::unique_ptr<NodeThread>                    pnode_thread_;
    rclcpp::TimerBase::SharedPtr                   update_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

    // ── Parameters ──────────────────────────────────────────────────────────
    std::unique_ptr<ParamListener> param_listener_;
    Params                         params_;

    // ── Filter state  (protected by filter_mutex_) ───────────────────────────
    mutable std::mutex          filter_mutex_;
    ESKFd                       eskf_;
    std::optional<rclcpp::Time> last_predict_time_;

    // ── Observation queues (each internally mutex-protected) ─────────────────
    std::unique_ptr<PoseObsQueue>  pose_queue_;
    std::unique_ptr<TwistObsQueue> twist_queue_;

    // ── Initialisation guard ─────────────────────────────────────────────────
    std::atomic<bool> is_initialized_{false};

    // ── Output cache  (protected by cached_state_mutex_) ────────────────────
    mutable std::shared_mutex cached_state_mutex_;
    NavStateD    cached_state_{};
    PMat         cached_covariance_ = PMat::Identity() * 1e-2;
    rclcpp::Time cached_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Logger logger_;
};

}  // namespace finenav

