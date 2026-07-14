// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <algorithm>
#include <deque>
#include <vector>

#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include "finenav_localizer/kalman_filter/filter_model.hpp"
#include "finenav_localizer/kalman_filter/time_delay_kalman_filter.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav::localizer {

/**
 * @brief ESKF<Model>  —  Error-State Kalman Filter with time-delay compensation
 * @details
 *      Keeps the error-state covariance inside a TimeDelayKalmanFilter.
 *      Each delayed observation linearises at the nominal from its own time
 * @tparam Model navigation-model descriptor: exposes the state / matrix types and
 *               the predictNominal / computeF / buildQ / applyErrorState / projectCovariance hooks.
 */
template <typename Model>
class ESKF {
public:
    using S         = typename Model::ScalarT;
    using NavStateT = typename Model::NavStateT;
    using ErrorVec  = typename Model::ErrorVec;
    using FMat      = typename Model::FMat;
    using QMat      = typename Model::QMat;
    using PMat      = typename Model::PMat;

    static constexpr int kDim = kNavStateDim;  ///< 12

    /**
     * @brief Construct the filter and seed every delay slot with init_cov.
     * @param model_opts process-noise parameters forwarded to Model::buildQ.
     * @param max_delay_step number of delay slots (≥ 1), observations older than this are rejected.
     * @param init_cov initial state covariance P₀.
     * @param logger ROS logger (injected from owner).
     */
    explicit ESKF(typename Model::Options model_opts   = {},
                  int                    max_delay_step = 50,
                  PMat                   init_cov       = PMat::Identity() * static_cast<S>(1e-2),
                  rclcpp::Logger         logger         = rclcpp::get_logger("finenav.ESKF"))
        : model_opts_(std::move(model_opts))
        , accumulated_delay_times_(static_cast<std::size_t>(max_delay_step), 1.0e15)
        , logger_(std::move(logger))
    {
        const Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.init(x0, init_cov.template cast<double>(), max_delay_step);
    }

    /**
     * @brief Propagate the nominal state and the error-state covariance by dt.
     * @param dt time step [s]; non-positive dt returns immediately (caller should validate).
     */
    void predict(S dt)
    {
        if (dt <= static_cast<S>(0.0)) {
            return;
        }

        // Compute F and Q from the current nominal state *before* propagating.
        const FMat F = Model::computeF(nominal_, static_cast<S>(dt));
        const QMat Q = Model::buildQ(model_opts_, static_cast<S>(dt));

        // ── Save nominal into history BEFORE propagation ────────────────────
        //  nominal_history_[0] will become delay_step=1 (one tick ago).
        //  The oldest slot beyond max_delay_step is discarded.
        nominal_history_.push_front(nominal_);
        if (static_cast<int>(nominal_history_.size()) >= tdkf_.maxDelayStep()) {
            nominal_history_.pop_back();
        }

        // Propagate nominal state.
        nominal_ = Model::predictNominal(nominal_, static_cast<S>(dt));

        // Propagate error-state covariance via time-delay extended structure.
        // x_next is zero: error state is always reset after each update().
        const Eigen::MatrixXd x_next = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.predictWithDelay(x_next,
                               F.template cast<double>(),
                               Q.template cast<double>());

        // Track elapsed time in each delay slot.
        accumulateDelayTime(static_cast<double>(dt));

        FINENAV_LOG_DEBUG(logger_,
            "predict  dt={:.4f}  |p|={:.3f}  |v|={:.3f}  |ω|={:.3f}",
            static_cast<double>(dt),
            static_cast<double>(nominal_.p.norm()),
            static_cast<double>(nominal_.v.norm()),
            static_cast<double>(nominal_.omega.norm()));
    }

    /**
     * @brief Generic delayed observation update.
     * @tparam ObsDim observation vector length.
     * @param innovation z − h(x̂)                   (ObsDim × 1)
     * @param H observation Jacobian                 (ObsDim × kDim)
     * @param R_noise observation-noise covariance   (ObsDim × ObsDim)
     * @param delay_step delay slot (0 = no delay, from findDelayStep()).
     * @return false if the observation is rejected.
     */
    template <int ObsDim>
    bool update(const Eigen::Matrix<S, ObsDim, 1>      & innovation,
                const Eigen::Matrix<S, ObsDim, kDim>   & H,
                const Eigen::Matrix<S, ObsDim, ObsDim> & R_noise,
                int                                      delay_step = 0)
    {
        // ── 1. Delayed KF update ────────────────────────────────────────────
        if (!tdkf_.updateWithDelay(innovation.template cast<double>(),
                                   H.template cast<double>(),
                                   R_noise.template cast<double>(),
                                   delay_step)) {
            return false;
        }

        // ── 2. Extract error-state correction from the leading block ────────
        const ErrorVec dx = tdkf_.getLatestX().template cast<S>();
        const S dx_norm   = dx.norm();  // capture for debug log

        // ── 3. Apply error corrections to ALL nominal states ────────────────
        nominal_ = Model::applyErrorState(nominal_, dx);

        const int n_hist = std::min(
            tdkf_.maxDelayStep() - 1,
            static_cast<int>(nominal_history_.size()));
        for (int i = 0; i < n_hist; ++i) {
            // TDKF slot index (i+1) ↔ nominal_history_[i]
            const ErrorVec dx_i =
                tdkf_.getXAtSlot(i + 1).template cast<S>();
            // Only bother if the correction is non-trivial (avoids unnecessary
            // SO3 exponentials when the cross-covariance contribution is zero).
            if (dx_i.squaredNorm() > static_cast<S>(1e-20)) {
                nominal_history_[static_cast<std::size_t>(i)] =
                    Model::applyErrorState(nominal_history_[static_cast<std::size_t>(i)],
                                          dx_i);
            }
        }

        // ── 4. ESKF covariance projection on the leading block ──────────────
        //  P(0,0) ← G · P(0,0) · Gᵀ   where G = I − 0.5·[δθ]×  (SO3 block)
        //  Cross-correlation blocks P(i,0) / P(0,j) for i,j ≥ 1 are not
        //  explicitly projected because G ≈ I for small δθ (second-order).
        const PMat P_lead      = tdkf_.getLatestP().template cast<S>();
        const PMat P_projected = Model::projectCovariance(P_lead, dx);
        tdkf_.setLeadingCovariance(P_projected.template cast<double>());

        // ── 5. Reset all error-state slots to zero (ESKF reset step) ────────
        tdkf_.resetState();

        FINENAV_LOG_DEBUG(logger_,
            "update  ObsDim={}  delay_step={}  |innov|={:.4f}  |dx|={:.4f}",
            ObsDim, delay_step,
            static_cast<double>(innovation.norm()),
            static_cast<double>(dx_norm));
        return true;
    }

   /**
     * @brief Dual-epoch observation update.
     * @tparam ObsDim observation vector length.
     * @param innovation z − h(x̂)                      (ObsDim × 1)
     * @param H_to Jacobian for the "to"   epoch         (ObsDim × kDim)
     * @param H_from Jacobian for the "from" epoch       (ObsDim × kDim)
     * @param R_noise observation-noise covariance       (ObsDim × ObsDim)
     * @param delay_step_to delay slot for the "to"   frame (≥ 0)
     * @param delay_step_from delay slot for the "from" frame (≥ delay_step_to)
     * @return false if the observation is rejected.
     */
    template <int ObsDim>
    bool update(const Eigen::Matrix<S, ObsDim, 1>      & innovation,
                const Eigen::Matrix<S, ObsDim, kDim>   & H_to,
                const Eigen::Matrix<S, ObsDim, kDim>   & H_from,
                const Eigen::Matrix<S, ObsDim, ObsDim> & R_noise,
                int                                      delay_step_to,
                int                                      delay_step_from)
    {
        // ── 1. Dual-slot KF update ──────────────────────────────────────────
        if (!tdkf_.updateWithTwoSlots(
                innovation.template cast<double>(),
                H_to.template cast<double>(),   delay_step_to,
                H_from.template cast<double>(), delay_step_from,
                R_noise.template cast<double>())) {
            return false;
        }

        // ── 2–5. Identical post-update logic as the single-block update ─────
        const ErrorVec dx     = tdkf_.getLatestX().template cast<S>();
        const S        dx_norm = dx.norm();

        nominal_ = Model::applyErrorState(nominal_, dx);

        const int n_hist = std::min(
            tdkf_.maxDelayStep() - 1,
            static_cast<int>(nominal_history_.size()));
        for (int i = 0; i < n_hist; ++i) {
            const ErrorVec dx_i = tdkf_.getXAtSlot(i + 1).template cast<S>();
            if (dx_i.squaredNorm() > static_cast<S>(1e-20)) {
                nominal_history_[static_cast<std::size_t>(i)] =
                    Model::applyErrorState(
                        nominal_history_[static_cast<std::size_t>(i)], dx_i);
            }
        }

        const PMat P_lead      = tdkf_.getLatestP().template cast<S>();
        const PMat P_projected = Model::projectCovariance(P_lead, dx);
        tdkf_.setLeadingCovariance(P_projected.template cast<double>());
        tdkf_.resetState();

        FINENAV_LOG_DEBUG(logger_,
            "dual update  ObsDim={}  to={}  from={}  "
            "|innov|={:.4f}  |dx|={:.4f}",
            ObsDim, delay_step_to, delay_step_from,
            static_cast<double>(innovation.norm()),
            static_cast<double>(dx_norm));
        return true;
    }

    /**
     * @brief Find the delay slot whose accumulated time is closest to delay_time_sec.
     * @param delay_time_sec observation age [s].
     * @return closest slot index, even if the age exceeds the maximum tracked delay.
     */
    [[nodiscard]] std::size_t findDelayStep(double delay_time_sec) const
    {
        // If larger than the oldest slot, signal rejection.
        if (delay_time_sec > accumulated_delay_times_.back()) {
            return accumulated_delay_times_.size();
        }

        auto lower = std::lower_bound(
            accumulated_delay_times_.begin(),
            accumulated_delay_times_.end(),
            delay_time_sec);

        if (lower == accumulated_delay_times_.begin()) {
            return 0u;
        }
        if (lower == accumulated_delay_times_.end()) {
            return accumulated_delay_times_.size() - 1u;
        }

        // Pick whichever neighbour is closer.
        const auto prev = lower - 1;
        const bool closer_to_prev =
            (delay_time_sec - *prev) < (*lower - delay_time_sec);

        return closer_to_prev
            ? static_cast<std::size_t>(std::distance(accumulated_delay_times_.begin(), prev))
            : static_cast<std::size_t>(std::distance(accumulated_delay_times_.begin(), lower));
    }

    /**
     * @brief Maximum delay time currently tracked [s]; older observations should be discarded.
     */
    [[nodiscard]] double maxTrackedDelaySec() const
    {
        return accumulated_delay_times_.empty() ? 0.0 : accumulated_delay_times_.back();
    }

    /**
     * @brief Current best-estimate nominal state.
     */
    [[nodiscard]] const NavStateT & getState() const { return nominal_; }

    /**
     * @brief Nominal state at a given delay step.
     * @param delay_step 0 = current nominal; d = nominal from d ticks ago (nominal_history_[d−1]).
     * @return the delayed nominal; falls back to the oldest available entry if d exceeds history depth.
     */
    [[nodiscard]] const NavStateT & getStateAtDelay(int delay_step) const
    {
        if (delay_step <= 0 || nominal_history_.empty()) {
            return nominal_;
        }
        const auto idx = static_cast<std::size_t>(delay_step - 1);
        if (idx < nominal_history_.size()) {
            return nominal_history_[idx];
        }
        return nominal_history_.back();  // oldest available
    }

    /**
     * @brief Covariance of the current (latest) error-state estimate — the leading block.
     */
    [[nodiscard]] PMat getCovariance() const
    {
        return tdkf_.getLatestP().template cast<S>();
    }

    /**
     * @brief Covariance block P(d,d) for a given delay step.
     * @param delay_step 0 = current covariance; d = covariance of the error-state d ticks ago.
     * @return P(d,d)
     */
    [[nodiscard]] PMat getCovarianceAtDelay(int delay_step) const
    {
        return tdkf_.getPAtSlot(delay_step).template cast<S>();
    }

    /**
     * @brief Overwrite the nominal state.
     */
    
    void setState(const NavStateT & state) { nominal_ = state; }
    void setState(NavStateT       && state) { nominal_ = std::move(state); }

    /**
     * @brief Overwrite the full extended covariance (all delay slots) with P.
     * @param P new state covariance applied to every slot.
     */
    void setCovariance(const PMat & P)
    {
        const Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.init(x0, P.template cast<double>(), tdkf_.maxDelayStep());
        accumulated_delay_times_.assign(
            static_cast<std::size_t>(tdkf_.maxDelayStep()), 1.0e15);
        nominal_history_.clear();
    }

    /**
     * @brief Reset the nominal state and the full extended covariance atomically.
     * @param state new nominal state.
     * @param P new state covariance.
     */
    void reinitialize(const NavStateT & state, const PMat & P)
    {
        nominal_ = state;
        setCovariance(P);  // also clears nominal_history_
    }

    /**
     * @brief Access / update the process-noise parameters.
     */
    [[nodiscard]] const typename Model::Options & getModelOptions() const { return model_opts_; }
    void setModelOptions(const typename Model::Options & opts)             { model_opts_ = opts; }

    /**
     * @brief Number of delay slots.
     */
    [[nodiscard]] int maxDelayStep() const { return tdkf_.maxDelayStep(); }

private:
    /**
     * @brief Shift accumulated_delay_times_ right, set slot 0 to 0, and add dt to the rest.
     * @param dt elapsed time since the previous predict [s].
     */
    void accumulateDelayTime(double dt)
    {
        std::copy_backward(
            accumulated_delay_times_.begin(),
            accumulated_delay_times_.end() - 1,
            accumulated_delay_times_.end());

        accumulated_delay_times_.front() = 0.0;
        for (std::size_t i = 1u; i < accumulated_delay_times_.size(); ++i) {
            accumulated_delay_times_[i] += dt;
        }
    }

    NavStateT               nominal_{};        ///< nominal (best-estimate) state
    std::deque<NavStateT, Eigen::aligned_allocator<NavStateT>>   nominal_history_; ///< Circular history of pre-propagation nominal states.
    TimeDelayKalmanFilter   tdkf_;             ///< extended error-state covariance
    typename Model::Options model_opts_;       ///< process-noise parameters
    std::vector<double>     accumulated_delay_times_;  ///< slot → elapsed time [s]
    rclcpp::Logger          logger_;
};

/// Convenience aliases.
using ESKFd = ESKF<NavigationModel<double>>;
using ESKFf = ESKF<NavigationModel<float>>;

}  // namespace finenav::localizer
