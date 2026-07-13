// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/eskf_localizer.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::localizer {

// ============================================================================
//  Construction / configuration
// ============================================================================
EskfLocalizer::EskfLocalizer(const Config & config)
    : logger_(rclcpp::get_logger("finenav.EskfLocalizer"))
{
    configure(config);
}

void EskfLocalizer::configure(const Config & config)
{
    config_ = config;

    NavigationModelD::Options model_opts;
    model_opts.vel_noise_std   = config_.vel_noise_std;
    model_opts.omega_noise_std = config_.omega_noise_std;

    // max_delay_step = ceil(max_delay_sec * update_rate_hz) + 1  (min 2).
    // maxTrackedDelaySec = (max_delay_step - 1) * dt, dt = 1/update_rate_hz.
    // Rule of thumb: max_delay_sec >= 2 / update_rate_hz (two periods of headroom).
    const double min_recommended_delay_sec = 2.0 / config_.update_rate_hz;
    if (config_.max_delay_sec < min_recommended_delay_sec) {
        FINENAV_LOG_WARN(
            logger_,
            "[EskfLocalizer] max_delay_sec={:.4f} s is less than the recommended minimum "
            "{:.4f} s (= 2 / update_rate_hz).  The effective rejection threshold equals one "
            "EKF period ({:.4f} s), so observations that arrive just after a tick may be "
            "rejected under scheduling jitter.  Consider max_delay_sec >= {:.4f} s.",
            config_.max_delay_sec, min_recommended_delay_sec,
            1.0 / config_.update_rate_hz, min_recommended_delay_sec);
    }

    const int max_delay_step = static_cast<int>(std::max(
        int64_t{2},
        static_cast<int64_t>(std::ceil(config_.max_delay_sec * config_.update_rate_hz)) + 1));
    eskf_ = ESKFd(model_opts, max_delay_step);

    pose_queue_ = std::make_unique<PoseObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, config_.pose_smoothing_steps)),
        static_cast<std::size_t>(config_.max_obs_queue_size));
    twist_queue_ = std::make_unique<TwistObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, config_.twist_smoothing_steps)),
        static_cast<std::size_t>(config_.max_obs_queue_size));
    rel_pose_queue_ = std::make_unique<RelativePoseObsQueue>(
        static_cast<std::size_t>(std::max(int64_t{1}, config_.rel_pose_smoothing_steps)),
        static_cast<std::size_t>(config_.max_rel_pose_queue_size));

    FINENAV_LOG_INFO(
        logger_,
        "[EskfLocalizer] Configured. rate={:.1f} Hz  max_delay={:.3f} s  max_delay_step={}  "
        "pose_smooth={}  twist_smooth={}",
        config_.update_rate_hz, config_.max_delay_sec, eskf_.maxDelayStep(),
        config_.pose_smoothing_steps, config_.twist_smoothing_steps);
}

void EskfLocalizer::setConfig(const Config & config)
{
    // Live-update the per-tick-read tunables (gates, smoothing, passthrough,
    // force_2d, max_dt, noise).  The estimator/queues are NOT rebuilt here, which
    // matches the previous behaviour where a dynamic parameter update only
    // refreshed the params struct read each tick.
    config_ = config;
}

// ============================================================================
//  setInitialPose  (thread-safe via filter_mutex_)
// ============================================================================
void EskfLocalizer::setInitialPose(const PoseMsg & msg)
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
        eskf_.reinitialize(init_state, P);
        pose_queue_->clear();
        twist_queue_->clear();
        rel_pose_queue_->clear();
        last_predict_time_.reset();
    }

    {
        std::unique_lock<std::shared_mutex> lk(cached_state_mutex_);
        cached_state_      = init_state;
        cached_covariance_ = P;
        cached_time_       = rclcpp::Time(msg.header.stamp);
    }


    is_initialized_.store(true, std::memory_order_release);

    FINENAV_LOG_INFO(logger_, "[EskfLocalizer] Initial pose set: p=[{:.3f}, {:.3f}, {:.3f}]",
        pos.x(), pos.y(), pos.z());
}

// ============================================================================
//  feedObservation  (thread-safe: queue has its own mutex)
// ============================================================================
void EskfLocalizer::feedObservation(const PoseMsg & msg)
{
    pose_queue_->push(msg);
}

void EskfLocalizer::feedObservation(const TwistMsg & msg)
{
    twist_queue_->push(msg);
}

void EskfLocalizer::feedRelativePose(const RelativePoseObs & obs)
{
    rel_pose_queue_->push(obs);
}

// ============================================================================
//  Getters  (shared_mutex — multiple readers allowed)
// ============================================================================
NavStateD EskfLocalizer::getState() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    NavStateD s = cached_state_;
    // External API contract: v is expressed in the body frame.
    s.v = s.R.inverse() * s.v;
    if (config_.force_2d_mode) {
        s = force2DOutput(s);
    }
    return s;
}

NavStateD EskfLocalizer::getRawState() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_state_;
}

EskfLocalizer::PMat EskfLocalizer::getCovariance() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_covariance_;
}

rclcpp::Time EskfLocalizer::getLastUpdateTime() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    return cached_time_;
}

// ============================================================================
//  step  — one estimator cycle (predict + drain queues + update + cache).
//          The sole writer of eskf_.  No TF / no timer / time injected as `now`.
// ============================================================================
void EskfLocalizer::step(const rclcpp::Time & now)
{
    // ==========================================
    // Case A: passthrough mode (zero calculation)
    // ==========================================
    if (config_.passthrough) {
        bool has_new_data = false;
        NavStateD next_state;

        std::lock_guard<std::mutex> lk(filter_mutex_);
        {
            std::unique_lock<std::shared_mutex> cache_lk(cached_state_mutex_);
            next_state = cached_state_;

            if (!pose_queue_->empty()) {
                PoseMsg latest_pose;
                while (!pose_queue_->empty()) {
                    latest_pose = pose_queue_->pop_increment_age();
                }
                auto [pos, rot, R_pose] = extractObservation(latest_pose);
                next_state.p = pos;
                next_state.R = rot;
                cached_covariance_.block<6, 6>(0, 0) = R_pose;
                has_new_data = true;
            }

            if (!twist_queue_->empty()) {
                TwistMsg latest_twist;
                while (!twist_queue_->empty()) {
                    latest_twist = twist_queue_->pop_increment_age();
                }
                auto [lin_v_body, ang_v_body, R_twist] = extractObservation(latest_twist);
                // Twist is body-frame; the cache stores v in world frame.
                next_state.v     = next_state.R * lin_v_body;
                next_state.omega = ang_v_body;
                cached_covariance_.block<6, 6>(6, 6) = R_twist;
                has_new_data = true;
            }

            if (has_new_data || is_initialized_.load(std::memory_order_acquire)) {
                cached_state_ = next_state;
                cached_time_  = now;
                // Keep the ESKF in sync so switching back to NORMAL doesn't jump.
                eskf_.setState(next_state);
            }
        }

        return;
    }

    // ==========================================
    // Case B: normal ESKF mode
    // ==========================================
    if (!is_initialized_.load(std::memory_order_acquire)) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "[EskfLocalizer] Not initialised — call setInitialPose() first.");
        return;
    }

    const rclcpp::Time & tick_time = now;
    std::lock_guard<std::mutex> lk(filter_mutex_);

    const double future_reject_threshold = -1.0 / config_.update_rate_hz;

    // ── 1. Compute dt ──────────────────────────────────────────────────────
    double dt = 1.0 / config_.update_rate_hz;  // nominal fallback (first cycle)
    if (last_predict_time_.has_value()) {
        const double meas_dt = (tick_time - *last_predict_time_).seconds();
        if (meas_dt < 0.0) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                "[EskfLocalizer] Time jumped backwards (dt={:.6f} s) — skipping cycle.", meas_dt);
            last_predict_time_ = tick_time;
            return;
        }
        if (meas_dt > config_.max_dt_sec) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                "[EskfLocalizer] Large dt ({:.4f} s > max {:.4f} s) — clamped.",
                meas_dt, config_.max_dt_sec);
            dt = config_.max_dt_sec;
        } else {
            dt = meas_dt;
        }
    }
    last_predict_time_ = tick_time;

    // ── 2. Predict ─────────────────────────────────────────────────────────
    eskf_.predict(dt);

    // ── 3. Process pose observations ───────────────────────────────────────
    {
        const std::size_t n = pose_queue_->size();
        for (std::size_t i = 0u; i < n; ++i) {
            const PoseMsg pose = pose_queue_->pop_increment_age();

            double delay_time = (tick_time - rclcpp::Time(pose.header.stamp)).seconds();
            if (delay_time < future_reject_threshold) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "[EskfLocalizer] Pose obs from future (delay={:.4f} s, threshold={:.4f} s) — skipping.",
                    delay_time, future_reject_threshold);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);
            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "[EskfLocalizer] Pose obs too old (delay={:.4f} s, max={:.4f} s) — rejecting.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(pose, static_cast<int>(delay_step));
        }
    }

    // ── 4. Process twist observations ──────────────────────────────────────
    {
        const std::size_t n = twist_queue_->size();
        for (std::size_t i = 0u; i < n; ++i) {
            const TwistMsg twist = twist_queue_->pop_increment_age();

            double delay_time = (tick_time - rclcpp::Time(twist.header.stamp)).seconds();
            if (delay_time < future_reject_threshold) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "[EskfLocalizer] Twist obs from future (delay={:.4f} s, threshold={:.4f} s) — skipping.",
                    delay_time, future_reject_threshold);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);
            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "[EskfLocalizer] Twist obs too old (delay={:.4f} s, max={:.4f} s) — rejecting.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(twist, static_cast<int>(delay_step));
        }
    }

    // ── 4.5 Process relative-pose (odometry) observations ──────────────────
    {
        const std::size_t n = rel_pose_queue_->size();
        for (std::size_t i = 0u; i < n; ++i) {
            const RelativePoseObs obs = rel_pose_queue_->pop_increment_age();

            double delay_to = (tick_time - obs.stamp_to).seconds();
            if (delay_to < future_reject_threshold) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "[EskfLocalizer] RelPose 'to' stamp from future (delay={:.4f} s) — skipping.", delay_to);
                continue;
            }
            delay_to = std::max(delay_to, 0.0);

            double delay_from = (tick_time - obs.stamp_from).seconds();
            if (delay_from < future_reject_threshold) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "[EskfLocalizer] RelPose 'from' stamp from future (delay={:.4f} s) — skipping.", delay_from);
                continue;
            }
            delay_from = std::max(delay_from, 0.0);

            const std::size_t ds_to   = eskf_.findDelayStep(delay_to);
            const std::size_t ds_from = eskf_.findDelayStep(delay_from);

            if (ds_to >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "[EskfLocalizer] RelPose 'to' frame too old (delay={:.4f} s) — rejecting.", delay_to);
                continue;
            }
            if (ds_from >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "[EskfLocalizer] RelPose 'from' frame too old (delay={:.4f} s, max={:.4f} s) — rejecting.",
                    delay_from, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(obs, static_cast<int>(ds_to), static_cast<int>(ds_from));
        }
    }

    // ── 5. Update output cache + history buffer ────────────────────────────
    {
        std::unique_lock<std::shared_mutex> cache_lk(cached_state_mutex_);
        cached_state_      = eskf_.getState();
        cached_covariance_ = eskf_.getCovariance();
        cached_time_       = tick_time;
    }
}

// ============================================================================
//  applyUpdate  — Mahalanobis gate + ESKF update with delay compensation
// ============================================================================
void EskfLocalizer::applyUpdate(const PoseMsg & msg, int delay_step)
{
    auto [pos_obs, rot_obs, R_raw] = extractObservation(msg);

    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(config_.pose_smoothing_steps);

    const auto obs = NavigationModelD::observePose(
        eskf_.getStateAtDelay(delay_step), pos_obs, rot_obs);

    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H * eskf_.getCovarianceAtDelay(delay_step) * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);

    FINENAV_LOG_INFO_THROTTLE(logger_, 2000, "Mahalanobis distance: {:.1f}", d2);

    if (d2 > config_.pose_gate_dist) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "[EskfLocalizer] applyUpdate<Pose> rejected by Mahalanobis gate "
            "(d²={:.4f} > threshold={:.4f}).", d2, config_.pose_gate_dist);
        return;
    }

    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);
}

void EskfLocalizer::applyUpdate(const TwistMsg & msg, int delay_step)
{
    auto [lin_vel, ang_vel, R_raw] = extractObservation(msg);

    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(config_.twist_smoothing_steps);

    const auto obs = NavigationModelD::observeVelocity(
        eskf_.getStateAtDelay(delay_step), lin_vel, ang_vel);

    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H * eskf_.getCovarianceAtDelay(delay_step) * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);

    if (d2 > config_.twist_gate_dist) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "[EskfLocalizer] applyUpdate<Twist> rejected by Mahalanobis gate "
            "(d²={:.4f} > threshold={:.4f}).", d2, config_.twist_gate_dist);
        return;
    }

    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);
}

void EskfLocalizer::applyUpdate(const RelativePoseObs & obs,
                                int delay_step_to, int delay_step_from)
{
    const Eigen::Matrix<double, 6, 6> R_eff =
        obs.R_noise * static_cast<double>(config_.rel_pose_smoothing_steps);

    const NavStateD from_state = eskf_.getStateAtDelay(delay_step_from);
    const NavStateD to_state   = eskf_.getStateAtDelay(delay_step_to);

    const auto dual_obs = NavigationModelD::observeRelativePose(
        from_state, to_state, obs.delta_p_body_obs, obs.delta_R_obs);

    const Eigen::Matrix<double, 6, 6> S_approx =
        dual_obs.H_to   * eskf_.getCovarianceAtDelay(delay_step_to)   * dual_obs.H_to.transpose()
      + dual_obs.H_from * eskf_.getCovarianceAtDelay(delay_step_from) * dual_obs.H_from.transpose()
      + R_eff;

    const double d2 = static_cast<double>(
        dual_obs.innovation.transpose() * S_approx.inverse() * dual_obs.innovation);

    if (d2 > config_.rel_pose_gate_dist) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "[EskfLocalizer] applyUpdate<RelPose> rejected by Mahalanobis gate "
            "(d²={:.4f} > threshold={:.4f}).", d2, config_.rel_pose_gate_dist);
        return;
    }

    eskf_.update<6>(dual_obs.innovation, dual_obs.H_to, dual_obs.H_from,
                    R_eff, delay_step_to, delay_step_from);
}

// ============================================================================
//  extractObservation  —  message → filter-ready quantities
// ============================================================================
std::tuple<EskfLocalizer::Vec3d, EskfLocalizer::SO3d, Eigen::Matrix<double, 6, 6>>
EskfLocalizer::extractObservation(const PoseMsg & msg)
{
    Vec3d pos{msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};

    const Eigen::Quaterniond q{
        msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y, msg.pose.pose.orientation.z};
    const SO3d rot{q.normalized().toRotationMatrix()};

    Eigen::Matrix<double, 6, 6> R;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            R(r, c) = msg.pose.covariance[static_cast<std::size_t>(r * 6 + c)];

    R = (R + R.transpose()) * 0.5;
    for (int i = 0; i < 6; ++i)
        R(i, i) = std::max(R(i, i), 1e-6);

    return {pos, rot, R};
}

std::tuple<EskfLocalizer::Vec3d, EskfLocalizer::Vec3d, Eigen::Matrix<double, 6, 6>>
EskfLocalizer::extractObservation(const TwistMsg & msg)
{
    Vec3d lin{msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z};
    Vec3d ang{msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z};

    Eigen::Matrix<double, 6, 6> R;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            R(r, c) = msg.twist.covariance[static_cast<std::size_t>(r * 6 + c)];

    R = (R + R.transpose()) * 0.5;
    for (int i = 0; i < 6; ++i)
        R(i, i) = std::max(R(i, i), 1e-6);

    return {lin, ang, R};
}

// ============================================================================
//  force2DOutput  —  zero out z, roll, pitch; keep only x, y, yaw
// ============================================================================
NavStateD EskfLocalizer::force2DOutput(const NavStateD & s) const
{
    NavStateD out = s;
    out.p.z() = 0.0;
    const Eigen::Vector3d x_axis = s.R.matrix().col(0);
    const double yaw = std::atan2(x_axis.y(), x_axis.x());
    out.R = SO3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix());
    return out;
}

}  // namespace finenav::localizer
