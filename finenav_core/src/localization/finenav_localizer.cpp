// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_core/localization/finenav_localizer.hpp"

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
                                   LocalizerOptions        opts)
    : parent_node_(std::move(parent_node))
    , opts_(opts)
    , eskf_(opts.model_opts)
    , pose_queue_(opts.max_pose_queue_size,  opts.pose_smoothing_steps)
    , twist_queue_(opts.max_twist_queue_size, opts.twist_smoothing_steps)
    , logger_(rclcpp::get_logger("finenav.FineNavLocalizer"))
{
    // Private sub-node — mirrors the MapServer pattern so the timer lives on
    // its own executor thread and doesn't block the parent node.
    pnode_ = rclcpp::Node::make_shared(
        "finenav_localizer",
        parent_node_->get_name(),
        rclcpp::NodeOptions{});
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    // TF broadcaster is bound to the private node so it uses the right clock.
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(pnode_);

    // Periodic update timer
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / opts_.update_rate_hz));

    update_timer_ = rclcpp::create_timer(
        pnode_, parent_node_->get_clock(), period,
        [this]() { timerCallback(); });

    RCLCPP_INFO(
        logger_,
        "[FineNavLocalizer] Created. rate=%.1f Hz  max_delay=%.3f s  "
        "pose_smooth=%d  twist_smooth=%d",
        opts_.update_rate_hz, opts_.max_delay_sec,
        opts_.pose_smoothing_steps, opts_.twist_smoothing_steps);
}

// ============================================================================
//  setInitialPose  (thread-safe via filter_mutex_)
// ============================================================================
void FineNavLocalizer::setInitialPose(const PoseMsg & msg)
{
    auto [pos, rot, R_obs] = extractPoseObservation(msg);

    NavStateD init_state;
    init_state.p     = pos;
    init_state.R     = rot;
    init_state.v     = Vec3d::Zero();
    init_state.omega = Vec3d::Zero();

    // Build P₀: pose blocks from the message covariance;
    // velocity/omega are unknown so their variance is set large.
    PMat P = PMat::Zero();
    P.block<3, 3>(0, 0) = R_obs.block<3, 3>(0, 0);             // position
    P.block<3, 3>(3, 3) = R_obs.block<3, 3>(3, 3);             // rotation
    P.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 100.0; // linear vel
    P.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 100.0; // angular vel

    {
        std::lock_guard<std::mutex> lk(filter_mutex_);
        eskf_.setState(init_state);
        eskf_.setCovariance(P);
        state_history_.clear();
        last_predict_time_.reset();
    }

    // Populate output cache immediately so getters return a valid state
    // even before the first timerCallback() fires.
    {
        std::unique_lock<std::shared_mutex> lk(cached_state_mutex_);
        cached_state_      = init_state;
        cached_covariance_ = P;
        cached_time_       = rclcpp::Time(msg.header.stamp);
    }

    // Release-store so timerCallback() sees the initialised filter state
    // before reading is_initialized_ = true.
    is_initialized_.store(true, std::memory_order_release);

    RCLCPP_INFO(
        logger_,
        "[FineNavLocalizer] Initial pose set: p=[%.3f, %.3f, %.3f]",
        pos.x(), pos.y(), pos.z());
}

// ============================================================================
//  feedPose / feedTwist  (lock-free relative to the filter)
// ============================================================================
void FineNavLocalizer::feedPose(const PoseMsg & msg)
{
    // TODO: checke frame_id
    pose_queue_.push(msg);
    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] feedPose queued  (queue=%zu)", pose_queue_.size());
}

void FineNavLocalizer::feedTwist(const TwistMsg & msg)
{
    // TODO: check frame_id
    twist_queue_.push(msg);
    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] feedTwist queued  (queue=%zu)", twist_queue_.size());
}

// ============================================================================
//  Getters  (shared_mutex — multiple readers never block each other)
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

// ============================================================================
//  timerCallback  — sole modifier of eskf_ and state_history_
// ============================================================================
void FineNavLocalizer::timerCallback()
{
    // Acquire-load: guarantees we see everything written before
    // setInitialPose()'s release-store.
    if (!is_initialized_.load(std::memory_order_acquire)) { // TODO: 既然 initialized 允许重复发动，应该默认从 0 初始化
        RCLCPP_WARN_THROTTLE(
            logger_, *pnode_->get_clock(), 2000,
            "[FineNavLocalizer] Not initialised — call setInitialPose() first.");
        return;
    }

    const rclcpp::Time now = parent_node_->now();

    // Lock for the entire update cycle; setInitialPose() will block here for
    // at most one cycle (~10 ms at 100 Hz), which is acceptable.
    std::lock_guard<std::mutex> lk(filter_mutex_);

    // ── 1. Compute dt ──────────────────────────────────────────────────────
    double dt = 1.0 / opts_.update_rate_hz;  // nominal fallback (first cycle)
    if (last_predict_time_.has_value()) {
        const double meas_dt = (now - *last_predict_time_).seconds();
        if (meas_dt < 0.0) {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Time jumped backwards (dt=%.6f s) — skipping cycle.",
                meas_dt);
            last_predict_time_ = now;
            return;
        }
        if (meas_dt > opts_.max_dt_sec) {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Large dt (%.4f s > max %.4f s) — clamped.",
                meas_dt, opts_.max_dt_sec);
            dt = opts_.max_dt_sec;
        } else {
            dt = meas_dt;
        }
    }
    last_predict_time_ = now;

    // ── 2. Predict ─────────────────────────────────────────────────────────
    RCLCPP_DEBUG(logger_, "[FineNavLocalizer] predict  dt=%.5f s", dt);
    eskf_.predict(dt);

    // ── 3. Save post-predict snapshot ──────────────────────────────────────
    state_history_.push_back({now, eskf_.getState(), eskf_.getCovariance()});

    // Prune snapshots older than max_delay + 1 s margin
    const double horizon = opts_.max_delay_sec + 1.0; // TODO:这个+1是在干嘛
    while (!state_history_.empty() &&
           (now - state_history_.front().stamp).seconds() > horizon)
    {
        state_history_.pop_front();
    }

    // ── 4. Pose measurement updates ────────────────────────────────────────
    {
        auto entries = pose_queue_.drainAll();
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] %zu pose observations to process.", entries.size());
        for (const auto & e : entries) {
            applyPoseObservation(e, now);
        }
    }

    // ── 5. Twist measurement updates ───────────────────────────────────────
    {
        auto entries = twist_queue_.drainAll();
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] %zu twist observations to process.", entries.size());
        for (const auto & e : entries) {
            applyTwistObservation(e, now);
        }
    }

    // ── 6. Update output cache ─────────────────────────────────────────────
    {
        std::unique_lock<std::shared_mutex> cache_lk(cached_state_mutex_);
        cached_state_      = eskf_.getState();
        cached_covariance_ = eskf_.getCovariance();
        cached_time_       = now;
    }

    // ── 7. Broadcast TF: map → base_link ───────────────────────────────────
    const NavStateD & s = eskf_.getState();

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

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] tick  |p|=%.4f  |v|=%.4f  |ω|=%.4f",
        s.p.norm(), s.v.norm(), s.omega.norm());
}

// Observations stamped via now() on another thread can arrive with a stamp
// that is marginally newer than timerCallback's now due to scheduler jitter.
// Delays in [-kFutureToleranceSec, 0) are clamped to 0.0; anything more
// negative is treated as a genuinely out-of-order message and discarded.
static constexpr double kFutureToleranceSec = 0.003;  // 3 ms

// ============================================================================
//  applyPoseObservation  (Approach B retrocorrection)
//  PRECONDITION: filter_mutex_ is held by the caller (timerCallback).
// ============================================================================
void FineNavLocalizer::applyPoseObservation(
    const ObservationQueue<PoseMsg>::Entry & entry,
    const rclcpp::Time & now)
{
    double delay = (now - entry.stamp).seconds();

    if (delay < 0.0) {
        if (delay >= -kFutureToleranceSec) {
            RCLCPP_DEBUG(logger_,
                "[FineNavLocalizer] Pose obs marginally in future (delay=%.4f s) — clamped to 0.",
                delay);
            delay = 0.0;
        } else {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Pose obs from future (delay=%.4f s) — skipping.", delay);
            return;
        }
    }
    if (delay > opts_.max_delay_sec) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] Pose obs too old (delay=%.4f s > max %.4f s) — discarding.",
            delay, opts_.max_delay_sec);
        return;
    }

    auto [pos_obs, rot_obs, R_raw] = extractPoseObservation(entry.msg);
    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(opts_.pose_smoothing_steps);

    // ── Build temp ESKF starting from the closest historical snapshot ───────
    const auto snap_opt    = findSnapshot(entry.stamp);
    const bool has_snap    = snap_opt.has_value();
    const PMat  base_cov   = has_snap ? snap_opt->covariance : eskf_.getCovariance();
    const NavStateD base_s = has_snap ? snap_opt->state      : eskf_.getState();
    const rclcpp::Time base_stamp = has_snap ? snap_opt->stamp : now;

    ESKFd temp(opts_.model_opts, base_cov);
    temp.setState(base_s);

    // Micro-predict from snapshot stamp to observation stamp
    const double micro_dt = (entry.stamp - base_stamp).seconds();
    if (micro_dt > 1e-6) {
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] Pose micro-predict dt=%.5f s", micro_dt);
        temp.predict(micro_dt);
    }

    // ── Mahalanobis gate ────────────────────────────────────────────────────
    const auto obs = NavigationModelD::observePose(temp.getState(), pos_obs, rot_obs);
    const Eigen::Matrix<double, 6, 6> S =
        obs.H * temp.getCovariance() * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S.inverse() * obs.innovation);

    if (d2 > opts_.pose_gate_dist) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] Pose obs rejected by Mahalanobis gate "
            "(d²=%.4f > threshold=%.4f).", d2, opts_.pose_gate_dist);
        return;
    }

    // ── Kalman update then replay-predict to now ────────────────────────────
    temp.update<6>(obs.innovation, obs.H, R_eff);

    const double replay_dt = (now - entry.stamp).seconds();
    if (replay_dt > 1e-6) {
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] Pose replay-predict dt=%.5f s", replay_dt);
        temp.predict(replay_dt);
    }

    eskf_.setState(temp.getState());
    eskf_.setCovariance(temp.getCovariance());

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] Pose update applied  delay=%.3f s  d²=%.4f  "
        "|δp|=%.4f  |δR|=%.4f",
        delay, d2,
        obs.innovation.head<3>().norm(),
        obs.innovation.tail<3>().norm());
}

// ============================================================================
//  applyTwistObservation  (Approach B retrocorrection)
//  PRECONDITION: filter_mutex_ is held by the caller (timerCallback).
// ============================================================================
void FineNavLocalizer::applyTwistObservation(
    const ObservationQueue<TwistMsg>::Entry & entry,
    const rclcpp::Time & now)
{
    double delay = (now - entry.stamp).seconds();

    if (delay < 0.0) {
        if (delay >= -kFutureToleranceSec) {
            RCLCPP_DEBUG(logger_,
                "[FineNavLocalizer] Twist obs marginally in future (delay=%.4f s) — clamped to 0.",
                delay);
            delay = 0.0;
        } else {
            RCLCPP_WARN(logger_,
                "[FineNavLocalizer] Twist obs from future (delay=%.4f s) — skipping.", delay);
            return;
        }
    }
    if (delay > opts_.max_delay_sec) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] Twist obs too old (delay=%.4f s > max %.4f s) — discarding.",
            delay, opts_.max_delay_sec);
        return;
    }

    auto [lin_vel, ang_vel, R_raw] = extractTwistObservation(entry.msg);
    const Eigen::Matrix<double, 6, 6> R_eff =
        R_raw * static_cast<double>(opts_.twist_smoothing_steps);

    // ── Build temp ESKF ─────────────────────────────────────────────────────
    const auto snap_opt    = findSnapshot(entry.stamp);
    const bool has_snap    = snap_opt.has_value();
    const PMat  base_cov   = has_snap ? snap_opt->covariance : eskf_.getCovariance();
    const NavStateD base_s = has_snap ? snap_opt->state      : eskf_.getState();
    const rclcpp::Time base_stamp = has_snap ? snap_opt->stamp : now;

    ESKFd temp(opts_.model_opts, base_cov);
    temp.setState(base_s);

    const double micro_dt = (entry.stamp - base_stamp).seconds();
    if (micro_dt > 1e-6) {
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] Twist micro-predict dt=%.5f s", micro_dt);
        temp.predict(micro_dt);
    }

    // ── Mahalanobis gate ────────────────────────────────────────────────────
    const auto obs = NavigationModelD::observeVelocity(temp.getState(), lin_vel, ang_vel);
    const Eigen::Matrix<double, 6, 6> S =
        obs.H * temp.getCovariance() * obs.H.transpose() + R_eff;
    const double d2 = static_cast<double>(
        obs.innovation.transpose() * S.inverse() * obs.innovation);

    if (d2 > opts_.twist_gate_dist) {
        RCLCPP_WARN(logger_,
            "[FineNavLocalizer] Twist obs rejected by Mahalanobis gate "
            "(d²=%.4f > threshold=%.4f).", d2, opts_.twist_gate_dist);
        return;
    }

    // ── Kalman update then replay-predict to now ────────────────────────────
    temp.update<6>(obs.innovation, obs.H, R_eff);

    const double replay_dt = (now - entry.stamp).seconds();
    if (replay_dt > 1e-6) {
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] Twist replay-predict dt=%.5f s", replay_dt);
        temp.predict(replay_dt);
    }

    eskf_.setState(temp.getState());
    eskf_.setCovariance(temp.getCovariance());

    RCLCPP_DEBUG(logger_,
        "[FineNavLocalizer] Twist update applied  delay=%.3f s  d²=%.4f  "
        "|δv|=%.4f  |δω|=%.4f",
        delay, d2,
        obs.innovation.head<3>().norm(),
        obs.innovation.tail<3>().norm());
}

// ============================================================================
//  findSnapshot
//  Returns the snapshot with the largest stamp ≤ target.
//  Uses std::upper_bound on the chronologically-ordered deque.
//  NOTE: must be called while filter_mutex_ is held.
// ============================================================================
std::optional<FineNavLocalizer::StateSnapshot>
FineNavLocalizer::findSnapshot(const rclcpp::Time & target) const
{
    if (state_history_.empty()) {
        return std::nullopt;
    }

    // upper_bound: iterator to first element with stamp STRICTLY > target.
    auto it = std::upper_bound(
        state_history_.begin(), state_history_.end(), target,
        [](const rclcpp::Time & t, const StateSnapshot & snap) {
            return t < snap.stamp;
        });

    if (it == state_history_.begin()) {
        // All snapshots are after target — return oldest as fallback.
        RCLCPP_DEBUG(logger_,
            "[FineNavLocalizer] findSnapshot: all entries after target, using oldest.");
        return state_history_.front();
    }

    --it;  // last entry with stamp ≤ target
    return *it;
}

// ============================================================================
//  extractPoseObservation
// ============================================================================
std::tuple<FineNavLocalizer::Vec3d,
           FineNavLocalizer::SO3d,
           Eigen::Matrix<double, 6, 6>>
FineNavLocalizer::extractPoseObservation(const PoseMsg & msg)
{
    // Position
    Vec3d pos{
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z};

    // Rotation — normalise before constructing SO3 to handle near-unit quaternions
    const Eigen::Quaterniond q{
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z};
    const SO3d rot{q.normalized().toRotationMatrix()};

    // Covariance: ROS layout is [x, y, z, Rx, Ry, Rz] row-major, 36 elements.
    Eigen::Matrix<double, 6, 6> R;
    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            R(r, c) = msg.pose.covariance[static_cast<std::size_t>(r * 6 + c)];

    // Symmetrize + enforce positive diagonal for robustness against noisy sensors
    R = (R + R.transpose()) * 0.5;
    for (int i = 0; i < 6; ++i)
        R(i, i) = std::max(R(i, i), 1e-9);

    return {pos, rot, R};
}

// ============================================================================
//  extractTwistObservation
//  Twist is expected in body frame (base_link), matching observeVelocity().
// ============================================================================
std::tuple<FineNavLocalizer::Vec3d,
           FineNavLocalizer::Vec3d,
           Eigen::Matrix<double, 6, 6>>
FineNavLocalizer::extractTwistObservation(const TwistMsg & msg)
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





