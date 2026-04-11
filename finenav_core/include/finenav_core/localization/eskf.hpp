// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <rclcpp/logging.hpp>

#include "finenav_core/localization/filter_model.hpp"
#include "finenav_core/localization/time_delay_kalman_filter.hpp"

namespace finenav {

// ============================================================================
//  ESKF<Model>  —  Error-State Kalman Filter with time-delay compensation
//
//  Internally, the error-state covariance is maintained inside a
//  TimeDelayKalmanFilter, which expands the state to:
//
//      x_ex = [ δx_k | δx_{k-1} | … | δx_{k-d+1} ]
//
//  This lets delayed observations update the covariance and nominal state
//  correctly via cross-covariance blocks, without storing past nominal states.
//
//  ── Template parameter `Model` must expose ─────────────────────────────────
//    typename Model::ScalarT          —  scalar type (float / double)
//    typename Model::NavStateT        —  nominal state struct
//    typename Model::ErrorVec         —  error-state column vector (kNavStateDim × 1)
//    typename Model::FMat             —  (kNavStateDim × kNavStateDim)
//    typename Model::QMat             —  (kNavStateDim × kNavStateDim)
//    typename Model::PMat             —  (kNavStateDim × kNavStateDim)
//    typename Model::Options          —  noise-parameter struct
//
//    Model::predictNominal(s, dt)     —  propagate nominal state forward
//    Model::computeF(s, dt)           —  error-state transition Jacobian F
//    Model::buildQ(opts, dt)          —  discrete process-noise matrix Q
//    Model::applyErrorState(s, dx)    —  nominal ← nominal ⊕ dx
//    Model::projectCovariance(P, dx)  —  ESKF covariance reset projection
//
//  ── Delay-handling design ──────────────────────────────────────────────────
//  • Every predict(dt) call slides all delay slots and accumulates elapsed
//    time in accumulated_delay_times_ (same mechanism as autoware EKFModule).
//  • findDelayStep(delay_sec) converts a wall-clock delay to the closest
//    integer delay slot index to pass to update(..., delay_step).
//  • update<ObsDim>(..., delay_step) calls updateWithDelay internally.
//    After the KF update:
//      1. dx   ← getLatestX() (the leading error-state block, block 0)
//      2. nominal ← applyErrorState(nominal, dx)
//      3. P(0,0) ← projectCovariance(P(0,0), dx)   [leading block only]
//      4. resetState() — zero all error-state slots in tdkf_
//
//  ── Thread safety ──────────────────────────────────────────────────────────
//  Not thread-safe.  All calls must be serialized by the owner
//  (FineNavLocalizer's timerCallback holds filter_mutex_).
//
//  This file is header-only (templates).
// ============================================================================
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

    // ---- Construction -----------------------------------------------------

    /// @param model_opts      Process-noise parameters forwarded to Model::buildQ.
    /// @param max_delay_step  Number of delay slots (≥ 1).
    ///                        Typically ceil(max_delay_sec * update_rate_hz).
    ///                        Observations older than this many ticks are rejected.
    /// @param init_cov        Initial state covariance P₀.
    explicit ESKF(typename Model::Options model_opts   = {},
                  int                    max_delay_step = 50,
                  PMat                   init_cov       = PMat::Identity() * static_cast<S>(1e-2))
        : model_opts_(std::move(model_opts))
        , accumulated_delay_times_(static_cast<std::size_t>(max_delay_step), 1.0e15)
        , logger_(rclcpp::get_logger("finenav.ESKF"))
    {
        const Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.init(x0, init_cov.template cast<double>(), max_delay_step);
    }

    // ---- Predict step -----------------------------------------------------
    //
    //  Nominal-state propagation + error-state covariance propagation.
    //
    //  Since the error state is always reset to zero after every update(),
    //  x_next passed to predictWithDelay is the zero vector.  Only the
    //  covariance (and its cross-correlation blocks) advances in time.
    //
    //  After predict(), accumulateDelayTime(dt) updates accumulated_delay_times_
    //  so that findDelayStep() returns meaningful indices.
    void predict(S dt)
    {
        if (dt <= static_cast<S>(0.0)) {
            RCLCPP_WARN(logger_,
                "[ESKF] predict() called with non-positive dt=%.6f — skipping.",
                static_cast<double>(dt));
            return;
        }

        // Compute F and Q from the current nominal state *before* propagating.
        const FMat F = Model::computeF(nominal_, static_cast<S>(dt));
        const QMat Q = Model::buildQ(model_opts_, static_cast<S>(dt));

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

        RCLCPP_DEBUG(logger_,
            "[ESKF] predict  dt=%.4f  |p|=%.3f  |v|=%.3f  |ω|=%.3f",
            static_cast<double>(dt),
            static_cast<double>(nominal_.p.norm()),
            static_cast<double>(nominal_.v.norm()),
            static_cast<double>(nominal_.omega.norm()));
    }

    // ---- Generic observation update  (compile-time observation dimension) -
    //
    //  @tparam ObsDim    Observation vector length.
    //  @param innovation  z − h(x̂)                       (ObsDim × 1)
    //  @param H           Observation Jacobian             (ObsDim × kDim)
    //  @param R_noise     Observation-noise covariance     (ObsDim × ObsDim)
    //  @param delay_step  Delay slot (0 = no delay, from findDelayStep()).
    //                     Default 0 is backward-compatible with callers that
    //                     do not yet pass a delay.
    //
    //  Returns false if updateWithDelay rejects the observation (e.g.
    //  delay_step ≥ max_delay_step) or if the KF gain is degenerate.
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

        // ── 3. Apply error-state correction to the nominal state ────────────
        nominal_ = Model::applyErrorState(nominal_, dx);

        // ── 4. ESKF covariance projection on the leading block ──────────────
        //  P(0,0) ← G · P(0,0) · Gᵀ   where G = I − 0.5·[δθ]×  (SO3 block)
        //  Cross-correlation blocks P(i,0) / P(0,j) for i,j ≥ 1 are not
        //  explicitly projected because G ≈ I for small δθ (second-order).
        const PMat P_lead      = tdkf_.getLatestP().template cast<S>();
        const PMat P_projected = Model::projectCovariance(P_lead, dx);
        tdkf_.setLeadingCovariance(P_projected.template cast<double>());

        // ── 5. Reset all error-state slots to zero (ESKF reset step) ────────
        tdkf_.resetState();

        RCLCPP_DEBUG(logger_,
            "[ESKF] update  ObsDim=%d  delay_step=%d  |innov|=%.4f  |dx|=%.4f",
            ObsDim, delay_step,
            static_cast<double>(innovation.norm()),
            static_cast<double>(dx_norm));
        return true;
    }

    // ---- Delay-step helpers (mirrors autoware EKFModule) ------------------

    /// Find the delay-slot index whose accumulated time is closest to
    /// delay_time_sec.  Returns accumulated_delay_times_.size() if
    /// delay_time_sec exceeds the maximum tracked delay (caller should
    /// reject the observation).
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

    /// Maximum delay time currently tracked (seconds).
    /// Observations older than this should be discarded.
    [[nodiscard]] double maxTrackedDelaySec() const
    {
        return accumulated_delay_times_.empty() ? 0.0 : accumulated_delay_times_.back();
    }

    // ---- Accessors --------------------------------------------------------
    [[nodiscard]] const NavStateT & getState()      const { return nominal_; }

    /// Returns the leading block of the extended covariance — i.e. the
    /// covariance of the CURRENT (latest) error-state estimate.
    [[nodiscard]] PMat getCovariance() const
    {
        return tdkf_.getLatestP().template cast<S>();
    }

    // ---- Mutators ---------------------------------------------------------

    void setState(const NavStateT & state) { nominal_ = state; }
    void setState(NavStateT       && state) { nominal_ = std::move(state); }

    /// Overwrites the full extended covariance (all delay slots) with P.
    /// Also resets accumulated_delay_times_ to 1e15 (all slots "unknown").
    /// Use this for a full initialisation / reinit after setInitialPose.
    void setCovariance(const PMat & P)
    {
        const Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(kDim, 1);
        tdkf_.init(x0, P.template cast<double>(), tdkf_.maxDelayStep());
        accumulated_delay_times_.assign(
            static_cast<std::size_t>(tdkf_.maxDelayStep()), 1.0e15);
    }

    /// Convenience: reset nominal state AND full extended covariance atomically.
    void reinitialize(const NavStateT & state, const PMat & P)
    {
        nominal_ = state;
        setCovariance(P);
    }

    [[nodiscard]] const typename Model::Options & getModelOptions() const { return model_opts_; }
    void setModelOptions(const typename Model::Options & opts)             { model_opts_ = opts; }

    [[nodiscard]] int maxDelayStep() const { return tdkf_.maxDelayStep(); }

private:
    // ---- Internal helpers -------------------------------------------------

    /// Shift accumulated_delay_times_ one slot to the right, insert 0.0 at
    /// slot 0, and add dt to all existing slots.  Mirrors autoware's
    /// EKFModule::accumulate_delay_time().
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

    // ---- Data members -----------------------------------------------------
    NavStateT               nominal_{};        ///< nominal (best-estimate) state
    TimeDelayKalmanFilter   tdkf_;             ///< extended error-state covariance
    typename Model::Options model_opts_;       ///< process-noise parameters
    std::vector<double>     accumulated_delay_times_;  ///< slot → elapsed time [s]
    rclcpp::Logger          logger_;
};

// ---- Convenience aliases --------------------------------------------------
using ESKFd = ESKF<NavigationModel<double>>;
using ESKFf = ESKF<NavigationModel<float>>;

}  // namespace finenav

