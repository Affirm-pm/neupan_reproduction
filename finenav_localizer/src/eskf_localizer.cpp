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


EskfLocalizer::EskfLocalizer(const finenav_localizer::Params & params,
                             rclcpp::Logger logger)
    : logger_(std::move(logger))
{
    configure(params);
}

void EskfLocalizer::configure(const finenav_localizer::Params & params)
{
    params_ = params;
    max_empty_ticks_ = static_cast<int>(params_.max_consecutive_empty_ticks);

    NavigationModelD::Options model_opts;
    model_opts.pos_noise_std   = params_.model.pos_noise_std;
    model_opts.rot_noise_std   = params_.model.rot_noise_std;
    model_opts.vel_noise_std   = params_.model.vel_noise_std;
    model_opts.omega_noise_std = params_.model.omega_noise_std;

    // max_delay_step = ceil(max_delay_sec * update_rate_hz) + 1  (min 2).
    // maxTrackedDelaySec = (max_delay_step - 1) * dt, dt = 1/update_rate_hz.
    // Rule of thumb: max_delay_sec >= 2 / update_rate_hz (two periods of headroom).
    const double min_recommended_delay_sec = 2.0 / params_.update_rate_hz;
    if (params_.max_delay_sec < min_recommended_delay_sec) {
        FINENAV_LOG_WARN(
            logger_,
            "max_delay_sec={:.4f} s is below the recommended minimum {:.4f} s "
            "(= 2 / update_rate_hz). Observations arriving just after a tick may be "
            "rejected under scheduling jitter. Consider max_delay_sec >= {:.4f} s.",
            params_.max_delay_sec, min_recommended_delay_sec,
            min_recommended_delay_sec);
    }

    const int max_delay_step = static_cast<int>(std::max(
        int64_t{2},
        static_cast<int64_t>(std::ceil(params_.max_delay_sec * params_.update_rate_hz)) + 1));
    eskf_ = ESKFd(model_opts, max_delay_step,
                  ESKFd::PMat::Identity() * static_cast<double>(1e-2),
                  logger_.get_child("ESKF"));

    pose_queue_ = std::make_unique<PoseObsQueue>(
        1u, static_cast<std::size_t>(params_.max_pose_queue_size));
    twist_queue_ = std::make_unique<TwistObsQueue>(
        1u, static_cast<std::size_t>(params_.max_twist_queue_size));
    rel_pose_queue_ = std::make_unique<RelativePoseObsQueue>(
        1u, static_cast<std::size_t>(params_.max_rel_pose_queue_size));

    FINENAV_LOG_INFO(
        logger_,
        "Configured. rate={:.1f} Hz  max_delay={:.3f} s  max_delay_step={}",
        params_.update_rate_hz, params_.max_delay_sec, eskf_.maxDelayStep());
}

void EskfLocalizer::setConfig(const finenav_localizer::Params & params)
{
    params_          = params;
    max_empty_ticks_ = static_cast<int>(params_.max_consecutive_empty_ticks);
}

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
    P.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1.0;   // linear vel (unknown, moderate initial confidence)
    P.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1.0;   // angular vel (unknown, moderate initial confidence)

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

    FINENAV_LOG_INFO(logger_, "Initial pose set at [{:.3f}, {:.3f}, {:.3f}]",
        pos.x(), pos.y(), pos.z());
}

void EskfLocalizer::feedObservation(const PoseMsg & msg)
{
    pose_queue_->push(TaggedPoseObs{msg});
}

void EskfLocalizer::feedObservation(const PoseMsg & msg, int smooth,
                                    double gate, const std::array<bool,6>& mask,
                                    const std::array<double,6>& cov_override)
{
    TaggedPoseObs t{msg, smooth, gate, mask};
    t.cov_override = cov_override;
    pose_queue_->push(std::move(t));
}

void EskfLocalizer::feedObservation(const TwistMsg & msg)
{
    twist_queue_->push(TaggedTwistObs{msg});
}

void EskfLocalizer::feedObservation(const TwistMsg & msg, int smooth,
                                    double gate, const std::array<bool,6>& mask,
                                    const std::array<double,6>& cov_override)
{
    TaggedTwistObs t{msg, smooth, gate, mask};
    t.cov_override = cov_override;
    twist_queue_->push(std::move(t));
}

void EskfLocalizer::feedRelativePose(const RelativePoseObs & obs)
{
    rel_pose_queue_->push(obs);
}

NavStateD EskfLocalizer::getState() const
{
    std::shared_lock<std::shared_mutex> lk(cached_state_mutex_);
    NavStateD s = cached_state_;
    // External API contract: v is expressed in the body frame.
    s.v = s.R.inverse() * s.v;
    if (params_.force_2d_mode) {
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


void EskfLocalizer::step(const rclcpp::Time & now)
{
    // ==========================================
    // Case A: passthrough mode (zero calculation)
    // ==========================================
    if (params_.passthrough) {
        bool has_new_data = false;
        NavStateD next_state;

        std::lock_guard<std::mutex> lk(filter_mutex_);
        {
            std::unique_lock<std::shared_mutex> cache_lk(cached_state_mutex_);
            next_state = cached_state_;

            if (!pose_queue_->empty()) {
                PoseMsg latest_pose;
                while (!pose_queue_->empty()) {
                    latest_pose = pose_queue_->pop_increment_age().msg;
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
                    latest_twist = twist_queue_->pop_increment_age().msg;
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
            "Not initialised — call setInitialPose() first.");
        return;
    }

    // ── Predict-guard: if ALL queues have been empty for too many consecutive
    //     ticks, freeze the filter (constant-velocity predict alone has no
    //     physical basis and the filter is better off holding its last good
    //     estimate until a real observation arrives).
    const bool all_empty = (pose_queue_->empty() && twist_queue_->empty() &&
                            rel_pose_queue_->empty());
    if (all_empty) {
        consecutive_empty_ticks_++;
        if (consecutive_empty_ticks_ == max_empty_ticks_) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 500,
                "All observation queues empty for {} consecutive ticks "
                "(≥ max {}) — freezing filter until an observation arrives.",
                consecutive_empty_ticks_, max_empty_ticks_);
        }
        if (consecutive_empty_ticks_ >= max_empty_ticks_) {
            // Frozen: only advance the output timestamp; state & covariance stay.
            std::unique_lock<std::shared_mutex> cache_lk(cached_state_mutex_);
            cached_time_ = now;
            return;
        }
    } else {
        if (consecutive_empty_ticks_ >= max_empty_ticks_) {
            FINENAV_LOG_INFO_THROTTLE(logger_, 500,
                "Observation arrived — unfreezing filter "
                "(was frozen for {} ticks).",
                consecutive_empty_ticks_);
        }
        if (consecutive_empty_ticks_ > 0) {
            // We skipped at least one predict — reset the dt reference so the
            // first predict after the gap uses the nominal dt, not the wall-clock
            // gap (which could be seconds and get clamped).
            last_predict_time_.reset();
        }
        consecutive_empty_ticks_ = 0;
    }

    const rclcpp::Time & tick_time = now;
    std::lock_guard<std::mutex> lk(filter_mutex_);

    const double future_reject_threshold = -1.0 / params_.update_rate_hz;

    // ── 1. Compute dt ──────────────────────────────────────────────────────
    double dt = 1.0 / params_.update_rate_hz;  // nominal fallback (first cycle)
    if (last_predict_time_.has_value()) {
        const double meas_dt = (tick_time - *last_predict_time_).seconds();
        if (meas_dt < 0.0) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                "Time jumped backwards (dt={:.6f} s) — skipping cycle.", meas_dt);
            last_predict_time_ = tick_time;
            return;
        }
        if (meas_dt > params_.max_dt_sec) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                "Large dt ({:.4f} s > max {:.4f} s) — clamped.",
                meas_dt, params_.max_dt_sec);
            dt = params_.max_dt_sec;
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
        if (n >= static_cast<std::size_t>(params_.max_pose_queue_size) * 8 / 10) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                "Pose observation queue near capacity ({}/{}) — consumer may be falling behind",
                n, params_.max_pose_queue_size);
        }
        for (std::size_t i = 0u; i < n; ++i) {
            const TaggedPoseObs tagged = pose_queue_->pop_increment_age();

            double delay_time = (tick_time - rclcpp::Time(tagged.msg.header.stamp)).seconds();
            if (delay_time < future_reject_threshold) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "Pose observation from future (delay={:.4f} s, threshold={:.4f} s) — skipping.",
                    delay_time, future_reject_threshold);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);
            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "Pose observation too old (delay={:.4f} s > max {:.4f} s) — rejected.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(tagged, static_cast<int>(delay_step));
        }
    }

    // ── 4. Process twist observations ──────────────────────────────────────
    {
        const std::size_t n = twist_queue_->size();
        if (n >= static_cast<std::size_t>(params_.max_twist_queue_size) * 8 / 10) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                "Twist observation queue near capacity ({}/{}) — consumer may be falling behind",
                n, params_.max_twist_queue_size);
        }
        for (std::size_t i = 0u; i < n; ++i) {
            const TaggedTwistObs tagged = twist_queue_->pop_increment_age();

            double delay_time = (tick_time - rclcpp::Time(tagged.msg.header.stamp)).seconds();
            if (delay_time < future_reject_threshold) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "Twist observation from future (delay={:.4f} s, threshold={:.4f} s) — skipping.",
                    delay_time, future_reject_threshold);
                continue;
            }
            delay_time = std::max(delay_time, 0.0);

            const std::size_t delay_step = eskf_.findDelayStep(delay_time);
            if (delay_step >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "Twist observation too old (delay={:.4f} s > max {:.4f} s) — rejected.",
                    delay_time, eskf_.maxTrackedDelaySec());
                continue;
            }

            applyUpdate(tagged, static_cast<int>(delay_step));
        }
    }

    // ── 4.5 Process relative-pose (odometry) observations ──────────────────
    {
        const std::size_t n = rel_pose_queue_->size();
        if (n >= static_cast<std::size_t>(params_.max_rel_pose_queue_size) * 8 / 10) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                "RelPose observation queue near capacity ({}/{}) — consumer may be falling behind",
                n, params_.max_rel_pose_queue_size);
        }
        for (std::size_t i = 0u; i < n; ++i) {
            const RelativePoseObs obs = rel_pose_queue_->pop_increment_age();

            double delay_to = (tick_time - obs.stamp_to).seconds();
            if (delay_to < future_reject_threshold) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "RelPose 'to' stamp from future (delay={:.4f} s) — skipping.", delay_to);
                continue;
            }
            delay_to = std::max(delay_to, 0.0);

            double delay_from = (tick_time - obs.stamp_from).seconds();
            if (delay_from < future_reject_threshold) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "RelPose 'from' stamp from future (delay={:.4f} s) — skipping.", delay_from);
                continue;
            }
            delay_from = std::max(delay_from, 0.0);

            const std::size_t ds_to   = eskf_.findDelayStep(delay_to);
            const std::size_t ds_from = eskf_.findDelayStep(delay_from);

            if (ds_to >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "RelPose 'to' frame too old (delay={:.4f} s > max {:.4f} s) — rejected.",
                    delay_to, eskf_.maxTrackedDelaySec());
                continue;
            }
            if (ds_from >= static_cast<std::size_t>(eskf_.maxDelayStep())) {
                FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                    "RelPose 'from' frame too old (delay={:.4f} s > max {:.4f} s) — rejected.",
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

namespace {
/// Apply per-sensor covariance overrides: R(i,i) ← override[i] when override[i] ≥ 0.
void applyCovOverride(Eigen::Matrix<double, 6, 6> & R,
                      const std::array<double, 6> & cov_override)
{
    for (int i = 0; i < 6; ++i) {
        if (cov_override[static_cast<std::size_t>(i)] >= 0.0) {
            R(i, i) = cov_override[static_cast<std::size_t>(i)];
        }
    }
}
}  // namespace

void EskfLocalizer::applyUpdate(const TaggedPoseObs & tagged, int delay_step)
{
    auto [pos_obs, rot_obs, R_raw] = extractObservation(tagged.msg);
    applyCovOverride(R_raw, tagged.cov_override);

    const int  k = static_cast<int>(std::count(tagged.mask.begin(), tagged.mask.end(), true));
    if (k == 0) return;

    const Eigen::Matrix<double, 6, 6> R_eff = R_raw * static_cast<double>(tagged.smooth);

    const auto obs = NavigationModelD::observePose(
        eskf_.getStateAtDelay(delay_step), pos_obs, rot_obs);

    if (k < 6) {
        auto [H_m, innov_m, R_m] = maskObservation(
            obs.H.template cast<double>(), obs.innovation.template cast<double>(),
            R_eff, tagged.mask);

        const Eigen::MatrixXd S_mat = H_m * eskf_.getCovarianceAtDelay(delay_step).template cast<double>() * H_m.transpose() + R_m;
        const double d2 = (innov_m.transpose() * S_mat.inverse() * innov_m)(0,0);
        FINENAV_LOG_DEBUG_THROTTLE(logger_, 2000,
            "Pose Mahalanobis d²={:.1f} (gate={:.1f})", d2, tagged.gate_dist);
        if (d2 > tagged.gate_dist) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                "Pose rejected by Mahalanobis gate (d²={:.1f} > {:.1f})", d2, tagged.gate_dist);
            return;
        }
        eskf_.updateDynamic(innov_m, H_m, R_m, delay_step);
        return;
    }

    // Fast path: all 6 DOFs active
    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H.template cast<double>() * eskf_.getCovarianceAtDelay(delay_step).template cast<double>() * obs.H.transpose().template cast<double>() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);
    FINENAV_LOG_DEBUG_THROTTLE(logger_, 2000,
        "Pose Mahalanobis d²={:.1f} (gate={:.1f})", d2, tagged.gate_dist);
    if (d2 > tagged.gate_dist) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "Pose rejected by Mahalanobis gate (d²={:.1f} > {:.1f})", d2, tagged.gate_dist);
        return;
    }
    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);
}

void EskfLocalizer::applyUpdate(const TaggedTwistObs & tagged, int delay_step)
{
    auto [lin_vel, ang_vel, R_raw] = extractObservation(tagged.msg);
    applyCovOverride(R_raw, tagged.cov_override);

    const int k = static_cast<int>(std::count(tagged.mask.begin(), tagged.mask.end(), true));
    if (k == 0) return;

    const Eigen::Matrix<double, 6, 6> R_eff = R_raw * static_cast<double>(tagged.smooth);

    const auto obs = NavigationModelD::observeVelocity(
        eskf_.getStateAtDelay(delay_step), lin_vel, ang_vel);

    if (k < 6) {
        auto [H_m, innov_m, R_m] = maskObservation(
            obs.H.template cast<double>(), obs.innovation.template cast<double>(),
            R_eff, tagged.mask);

        const Eigen::MatrixXd S_mat = H_m * eskf_.getCovarianceAtDelay(delay_step).template cast<double>() * H_m.transpose() + R_m;
        const double d2 = (innov_m.transpose() * S_mat.inverse() * innov_m)(0,0);
        FINENAV_LOG_DEBUG_THROTTLE(logger_, 2000,
            "Twist Mahalanobis d²={:.1f} (gate={:.1f})", d2, tagged.gate_dist);
        if (d2 > tagged.gate_dist) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
                "Twist rejected by Mahalanobis gate (d²={:.1f} > {:.1f})", d2, tagged.gate_dist);
            return;
        }
        eskf_.updateDynamic(innov_m, H_m, R_m, delay_step);
        return;
    }

    const Eigen::Matrix<double, 6, 6> S_mat =
        obs.H.template cast<double>() * eskf_.getCovarianceAtDelay(delay_step).template cast<double>() * obs.H.transpose().template cast<double>() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S_mat.inverse() * obs.innovation);
    FINENAV_LOG_DEBUG_THROTTLE(logger_, 2000,
        "Twist Mahalanobis d²={:.1f} (gate={:.1f})", d2, tagged.gate_dist);
    if (d2 > tagged.gate_dist) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "Twist rejected by Mahalanobis gate (d²={:.1f} > {:.1f})", d2, tagged.gate_dist);
        return;
    }
    eskf_.update<6>(obs.innovation, obs.H, R_eff, delay_step);
}

void EskfLocalizer::applyUpdate(const RelativePoseObs & obs,
                                int delay_step_to, int delay_step_from)
{
    const Eigen::Matrix<double, 6, 6> R_eff =
        obs.R_noise * static_cast<double>(obs.smooth);

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

    FINENAV_LOG_DEBUG_THROTTLE(logger_, 2000,
        "RelPose Mahalanobis d²={:.1f} (gate={:.1f})", d2, obs.gate_dist);

    if (d2 > obs.gate_dist) {
        FINENAV_LOG_WARN_THROTTLE(logger_, 2000,
            "RelPose rejected by Mahalanobis gate (d²={:.1f} > {:.1f})",
            d2, obs.gate_dist);
        return;
    }

    eskf_.update<6>(dual_obs.innovation, dual_obs.H_to, dual_obs.H_from,
                    R_eff, delay_step_to, delay_step_from);
}

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

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
EskfLocalizer::maskObservation(const Eigen::Matrix<double, 6, 12>& H_full,
                               const Eigen::Matrix<double, 6,  1>& innov_full,
                               const Eigen::Matrix<double, 6,  6>& R_full,
                               const std::array<bool, 6>&           config)
{
    const int k = static_cast<int>(std::count(config.begin(), config.end(), true));
    Eigen::MatrixXd H_m(k, 12);
    Eigen::MatrixXd innov_m(k, 1);
    Eigen::MatrixXd R_m(k, k);
    int row = 0;
    for (int i = 0; i < 6; ++i) {
        if (!config[static_cast<std::size_t>(i)]) continue;
        H_m.row(row)     = H_full.row(i);
        innov_m(row)     = innov_full(i);
        R_m(row, row)    = R_full(i, i);
        for (int j = 0, col = 0; j < 6; ++j) {
            if (!config[static_cast<std::size_t>(j)]) continue;
            R_m(row, col) = R_full(i, j);
            ++col;
        }
        ++row;
    }
    return {H_m, innov_m, R_m};
}

void EskfLocalizer::applyPlanarConstraints()
{
    constexpr double kSigmaZ     = 1e-4;  // [m]   — tight z=0 constraint
    constexpr double kSigmaRP    = 1e-3;  // [rad] — soft roll/pitch=0 constraint

    const auto cons = NavigationModelD::observePlanarConstraints(eskf_.getState());

    Eigen::Matrix<double, 3, 3> R_cons = Eigen::Matrix<double, 3, 3>::Zero();
    R_cons(0, 0) = kSigmaZ  * kSigmaZ;
    R_cons(1, 1) = kSigmaRP * kSigmaRP;
    R_cons(2, 2) = kSigmaRP * kSigmaRP;

    eskf_.updateDynamic(cons.innovation.template cast<double>(),
                        cons.H.template cast<double>(),
                        R_cons, 0);
}

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
