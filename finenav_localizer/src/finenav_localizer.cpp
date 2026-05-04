// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/finenav_localizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace finenav {

// ============================================================================
//  Constructor
// ============================================================================
FineNavLocalizer::FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                                   rclcpp::NodeOptions pnode_overrides)
    : parent_node_(std::move(parent_node))
    , logger_(rclcpp::get_logger("finenav.FineNavLocalizer"))
{
    // ── 创建私有子节点（参数前缀由 yaml 顶层 key "finenav_localizer" 决定）──
    pnode_overrides.append_parameter_override(
        "use_sim_time",
        parent_node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(
        "finenav_localizer",
        parent_node_->get_name(),
        pnode_overrides);

    // ── 读取参数 ──────────────────────────────────────────────────────────────
    param_listener_ = std::make_unique<ParamListener>(pnode_);
    params_ = param_listener_->get_params();

    // ── 初始化 ESKF ──────────────────────────────────────────────────────────
    NavigationModelD::Options model_opts;
    model_opts.vel_noise_std   = params_.model.vel_noise_std;
    model_opts.omega_noise_std = params_.model.omega_noise_std;

    const int max_delay_step = static_cast<int>(std::max(
        int64_t{1},
        static_cast<int64_t>(std::ceil(params_.max_delay_sec * params_.update_rate_hz))));
    eskf_ = ESKFd(model_opts, max_delay_step);

    pose_queue_  = std::make_unique<PoseObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, params_.pose_smoothing_steps)),
        static_cast<std::size_t>(params_.max_obs_queue_size));
    twist_queue_ = std::make_unique<TwistObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, params_.twist_smoothing_steps)),
        static_cast<std::size_t>(params_.max_obs_queue_size));

    // ── State history buffer ──────────────────────────────────────────────────
    state_buffer_ = std::make_shared<NavStateBuffer>(params_.history_duration_sec);

    // ── TF broadcaster (only if publish_tf is enabled) ───────────────────────
    if (params_.publish_tf) {
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(pnode_);
    }

    // ── 更新定时器 ────────────────────────────────────────────────────────────
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / params_.update_rate_hz));
    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period,
        [this]() { timerCallback(); });

    // ── 启动 spin 线程（最后执行，确保所有订阅/服务已注册）────────────────────
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    RCLCPP_INFO(
        logger_,
        "[FineNavLocalizer] Created. rate=%.1f Hz  max_delay=%.3f s  "
        "max_delay_step=%d  pose_smooth=%ld  twist_smooth=%ld",
        params_.update_rate_hz, params_.max_delay_sec,
        eskf_.maxDelayStep(),
        params_.pose_smoothing_steps, params_.twist_smoothing_steps);
}

// ============================================================================
//  setInitialPose  (thread-safe via filter_mutex_)
// ============================================================================
void FineNavLocalizer::setInitialPose(const PoseMsg & msg)
{
    auto [pos, rot, R_obs] = extractObservation(msg);

    NavStateD init_state;
    init_state.p     = pos;
    init_state.R     = rot;
    init_state.v     = Vec3d::Zero();
    init_state.omega = Vec3d::Zero();

    PMat P = PMat::Zero();
    P.block<3, 3>(0, 0) = R_obs.block<3, 3>(0, 0);              // position
    P.block<3, 3>(3, 3) = R_obs.block<3, 3>(3, 3);              // rotation
    P.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 100.0;  // linear vel (unknown)
    P.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 100.0;  // angular vel (unknown)

    {
        std::lock_guard<std::mutex> lk(filter_mutex_);
        // reinitialize() resets nominal state + ALL delay slots in TimeDelayKF
        // and resets accumulated_delay_times_ to 1e15.
        eskf_.reinitialize(init_state, P);
        pose_queue_->clear();
        twist_queue_->clear();
        last_predict_time_.reset();
    }

    {
        std::unique_lock<std::shared_mutex> lk(cached_state_mutex_);
        cached_state_      = init_state;
        cached_covariance_ = P;
        cached_time_       = rclcpp::Time(msg.header.stamp);
    }

    state_buffer_->clear();
    state_buffer_->push(rclcpp::Time(msg.header.stamp), init_state);

    is_initialized_.store(true, std::memory_order_release);

    RCLCPP_INFO(
        logger_,
        "[FineNavLocalizer] Initial pose set: p=[%.3f, %.3f, %.3f]",
        pos.x(), pos.y(), pos.z());
}

// ============================================================================
//  feedObservation  (thread-safe: queue has its own mutex)
// ============================================================================
void FineNavLocalizer::feedObservation(const PoseMsg & msg)
{
    pose_queue_->push(msg);
    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] feedObservation<Pose>  queued (queue=%zu)",
        pose_queue_->size());
}

void FineNavLocalizer::feedObservation(const TwistMsg & msg)
{
    twist_queue_->push(msg);
    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] feedObservation<Twist> queued (queue=%zu)",
        twist_queue_->size());
}

// ============================================================================
//  Getters  (shared_mutex — multiple readers allowed)
// ============================================================================
NavStateD FineNavLocalizer::getState() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_state_;
}

FineNavLocalizer::PMat FineNavLocalizer::getCovariance() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_covariance_;
}

rclcpp::Time FineNavLocalizer::getLastUpdateTime() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_time_;
}

std::optional<NavStateD> FineNavLocalizer::getStateAt(const rclcpp::Time & query_time) const
{
    return state_buffer_->getStateAt(query_time);
}

// ============================================================================
//  timerCallback  — the sole writer of eskf_
// ============================================================================
void FineNavLocalizer::timerCallback()
{
    if (!is_initialized_.load(std::memory_order_acquire)) {
        RCLCPP_WARN_THROTTLE(
            logger_, *pnode_->get_clock(), 2000,
            "[FineNavLocalizer] Not initialised — call setInitialPose() first.");
        return;
    }

    const rclcpp::Time now = pnode_->now();
    std::lock_guard<std::mutex> lk(filter_mutex_);

    // ── 1. Compute dt ──────────────────────────────────────────────────────
    double dt = 1.0 / params_.update_rate_hz;  // nominal fallback (first cycle)
    if (last_predict_time_.has_value()) {
        const double meas_dt = (now - *last_predict_time_).seconds();
        if (meas_dt < 0.0) {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Time jumped backwards (dt=%.6f s) — skipping cycle.",
                meas_dt);
            last_predict_time_ = now;
            return;
        }
        if (meas_dt > params_.max_dt_sec) {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Large dt (%.4f s > max %.4f s) — clamped.",
                meas_dt, params_.max_dt_sec);
            dt = params_.max_dt_sec;
        } else {
            dt = meas_dt;
        }
    }
    last_predict_time_ = now;

    // ── 2. Predict ─────────────────────────────────────────────────────────
    // eskf_.predict() internally propagates the nominal state, slides the
    // TimeDelayKF extended covariance, and updates accumulated_delay_times_.
    RCLCPP_DEBUG(logger_, "[FineNavLocalizer] predict  dt=%.5f s", dt);
    eskf_.predict(dt);

    // ── 3. Process pose observations ───────────────────────────────────────
    // Capture size before the loop: pop_increment_age() may re-enqueue at the
    // back, but we process only the items that were present at the start of
    // this tick.
    {
        const std::size_t n = pose_queue_->size();
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] pose observations this tick: %zu", n);

        for (std::size_t i = 0u; i < n; ++i) {
            const PoseMsg pose = pose_queue_->pop_increment_age();

            double delay_time = (now - rclcpp::Time(pose.header.stamp)).seconds();

            // Tolerate small scheduler jitter: clamp near-zero negative delays.
            if (delay_time < -0.003) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Pose obs from future (delay=%.4f s) — skipping.",
                    delay_time);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);
            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Pose obs too old (delay=%.4f s, "
                    "max tracked=%.4f s) — rejecting.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(pose, static_cast<int>(delay_step));
        }
    }

    // ── 4. Process twist observations ──────────────────────────────────────
    {
        const std::size_t n = twist_queue_->size();
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] twist observations this tick: %zu", n);

        for (std::size_t i = 0u; i < n; ++i) {
            const TwistMsg twist = twist_queue_->pop_increment_age();

            double delay_time = (now - rclcpp::Time(twist.header.stamp)).seconds();

            if (delay_time < -0.003) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Twist obs from future (delay=%.4f s) — skipping.",
                    delay_time);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);
            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                RCLCPP_WARN(logger_,
                    "[FineNavLocalizer] Twist obs too old (delay=%.4f s, "
                    "max tracked=%.4f s) — rejecting.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(twist, static_cast<int>(delay_step));
        }
    }

    // ── 5. Update output cache ─────────────────────────────────────────────
    {
        std::unique_lock<std::shared_mutex> cache_lk(cached_state_mutex_);
        cached_state_      = eskf_.getState();
        cached_covariance_ = eskf_.getCovariance();
        cached_time_       = now;
    }

    // ── 5b. Append to history buffer ──────────────────────────────────────
    state_buffer_->push(now, eskf_.getState());

    // ── 6. Broadcast TF: map → base_link ───────────────────────────────────
    const NavStateD & s = eskf_.getState();

    if (tf_br_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now;
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

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] tick  |p|=%.4f  |v|=%.4f  |ω|=%.4f",
        s.p.norm(), s.v.norm(), s.omega.norm());
}

// ============================================================================
//  applyUpdate  — Mahalanobis gate + ESKF update with delay compensation
// ============================================================================
void FineNavLocalizer::applyUpdate(const PoseMsg & msg, int delay_step)
{
    auto [pos_obs, rot_obs, R_raw] = extractObservation(msg);

    // Scale noise covariance by smoothing_steps: makes each of the N
    // processing steps less aggressive, resulting in a smooth correction
    // spread over N ticks.
    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(params_.pose_smoothing_steps);

    // Linearise at the nominal state from the time of capture, not the
    // current nominal.  This gives the correct innovation z − h(x̂_{k−d})
    // and the correct H Jacobian for the delayed KF update.
    const auto obs = NavigationModelD::observePose(
        eskf_.getStateAtDelay(delay_step), pos_obs, rot_obs);

    // ── Mahalanobis gate ────────────────────────────────────────────────────
    // Gate uses the CURRENT covariance (leading block of extended P).
    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H * eskf_.getCovariance() * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);

    if (d2 > params_.pose_gate_dist) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] applyUpdate<Pose> rejected by Mahalanobis gate "
            "(d²=%.4f > threshold=%.4f).", d2, params_.pose_gate_dist);
        return;
    }

    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] applyUpdate<Pose>   delay_step=%d  d²=%.4f  "
        "|δp|=%.4f  |δR|=%.4f",
        delay_step, d2,
        obs.innovation.head<3>().norm(), obs.innovation.tail<3>().norm());
}

void FineNavLocalizer::applyUpdate(const TwistMsg & msg, int delay_step)
{
    auto [lin_vel, ang_vel, R_raw] = extractObservation(msg);

    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(params_.twist_smoothing_steps);

    // Same as pose: linearise at the historical nominal from delay_step ticks ago.
    const auto obs = NavigationModelD::observeVelocity(
        eskf_.getStateAtDelay(delay_step), lin_vel, ang_vel);

    // ── Mahalanobis gate ────────────────────────────────────────────────────
    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H * eskf_.getCovariance() * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);

    if (d2 > params_.twist_gate_dist) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] applyUpdate<Twist> rejected by Mahalanobis gate "
            "(d²=%.4f > threshold=%.4f).", d2, params_.twist_gate_dist);
        return;
    }

    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] applyUpdate<Twist>  delay_step=%d  d²=%.4f  "
        "|δv|=%.4f  |δω|=%.4f",
        delay_step, d2,
        obs.innovation.head<3>().norm(), obs.innovation.tail<3>().norm());
}

// ============================================================================
//  extractObservation  —  message → filter-ready quantities
// ============================================================================
std::tuple<FineNavLocalizer::Vec3d,
           FineNavLocalizer::SO3d,
           Eigen::Matrix<double, 6, 6>>
FineNavLocalizer::extractObservation(const PoseMsg & msg)
{
    Vec3d pos{
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z};

    const Eigen::Quaterniond q{
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z};
    const SO3d rot{q.normalized().toRotationMatrix()};

    Eigen::Matrix<double, 6, 6> R;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            R(r, c) = msg.pose.covariance[static_cast<std::size_t>(r * 6 + c)];

    R = (R + R.transpose()) * 0.5;
    for (int i = 0; i < 6; ++i)
        R(i, i) = std::max(R(i, i), 1e-9);

    return {pos, rot, R};
}

std::tuple<FineNavLocalizer::Vec3d,
           FineNavLocalizer::Vec3d,
           Eigen::Matrix<double, 6, 6>>
FineNavLocalizer::extractObservation(const TwistMsg & msg)
{
    Vec3d lin{
        msg.twist.twist.linear.x,
        msg.twist.twist.linear.y,
        msg.twist.twist.linear.z};

    Vec3d ang{
        msg.twist.twist.angular.x,
        msg.twist.twist.angular.y,
        msg.twist.twist.angular.z};

    Eigen::Matrix<double, 6, 6> R;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            R(r, c) = msg.twist.covariance[static_cast<std::size_t>(r * 6 + c)];

    R = (R + R.transpose()) * 0.5;
    for (int i = 0; i < 6; ++i)
        R(i, i) = std::max(R(i, i), 1e-9);

    return {lin, ang, R};
}

}  // namespace finenav


