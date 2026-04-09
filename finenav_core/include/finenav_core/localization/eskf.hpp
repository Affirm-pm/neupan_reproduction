// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <rclcpp/logging.hpp>

#include "finenav_core/localization/filter_model.hpp"

namespace finenav {

// ============================================================================
//  ESKF<Model>  —  Error-State Kalman Filter driven by a model descriptor.
//
//  The template parameter `Model` must satisfy the following concept
//  (all members are static, so no instance is needed):
//
//    typename Model::ScalarT          —  scalar type (float / double)
//    typename Model::NavStateT        —  nominal state struct
//    typename Model::ErrorVec         —  error-state column vector (kNavStateDim × 1)
//    typename Model::FMat             —  state-transition Jacobian matrix
//    typename Model::QMat             —  process-noise covariance matrix
//    typename Model::PMat             —  state covariance matrix
//    typename Model::Options          —  noise-parameter struct
//
//    Model::predictNominal(s, dt)     —  propagate nominal state
//    Model::computeF(s, dt)          —  error-state transition Jacobian
//    Model::buildQ(opts, dt)         —  discrete process-noise matrix
//    Model::applyErrorState(s, dx)   —  fuse error correction into nominal
//    Model::projectCovariance(P, dx) —  ESKF covariance projection
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

    /// @param model_opts  Process-noise parameters forwarded to Model::buildQ.
    /// @param init_cov    Initial state covariance P₀.
    explicit ESKF(typename Model::Options model_opts = {},
                  PMat init_cov = PMat::Identity() * static_cast<S>(1e-2))
        : model_opts_(std::move(model_opts))
        , cov_(std::move(init_cov))
        , logger_(rclcpp::get_logger("finenav.ESKF")) // TODO: 文件头静态 Logger
    {}

    // ---- Predict step (dead-reckoning / propagation) ----------------------
    //
    //  1. Propagate nominal state:   x̂  ←  f(x̂, dt)
    //  2. Propagate error-state covariance:  P  ←  F·P·Fᵀ + Q
    //
    //  NOTE: the error state dx_ is **always** zero between updates (it is
    //  reset inside update()), so the propagation dx_ = F·dx_ is skipped.
    void predict(S dt) {
        if (dt <= static_cast<S>(0.0)) {
            RCLCPP_WARN(logger_,
                        "[ESKF] predict() called with non-positive dt=%.6f — skipping.",
                        static_cast<double>(dt));
            return;
        }

        // Compute F from the *current* nominal state before propagating.
        const FMat F = Model::computeF(nominal_, dt);
        const QMat Q = Model::buildQ(model_opts_, dt);

        nominal_ = Model::predictNominal(nominal_, dt);
        cov_     = F * cov_.eval() * F.transpose() + Q;

        RCLCPP_DEBUG(logger_,
                     "[ESKF] predict  dt=%.4f  |p|=%.3f  |v|=%.3f  |ω|=%.3f",
                     static_cast<double>(dt),
                     static_cast<double>(nominal_.p.norm()),
                     static_cast<double>(nominal_.v.norm()),
                     static_cast<double>(nominal_.omega.norm()));
    }

    // ---- Generic observation update  (compile-time observation dimension) -
    //
    //  @tparam ObsDim   Observation vector length (compile-time constant).
    //  @param innovation  Residual  z − h(x̂)            (ObsDim × 1)
    //  @param H           Observation Jacobian            (ObsDim × kDim)
    //  @param R_noise     Observation-noise covariance    (ObsDim × ObsDim)
    //
    //  After the update the error state is applied to the nominal state and
    //  then reset to zero (standard ESKF reset step).
    template <int ObsDim>
    bool update(const Eigen::Matrix<S, ObsDim, 1>&      innovation,
                const Eigen::Matrix<S, ObsDim, kDim>&   H,
                const Eigen::Matrix<S, ObsDim, ObsDim>& R_noise)
    {
        // Innovation covariance:  Sₖ = H·P·Hᵀ + R
        const Eigen::Matrix<S, ObsDim, ObsDim> S_mat =
            H * cov_ * H.transpose() + R_noise;

        // Kalman gain:  K = P·Hᵀ·Sₖ⁻¹
        const Eigen::Matrix<S, kDim, ObsDim> K =
            cov_ * H.transpose() * S_mat.inverse();

        // Error-state update and covariance update
        dx_  = K * innovation;
        cov_ = (FMat::Identity() - K * H) * cov_;

        const S dx_norm = static_cast<S>(dx_.norm());  // capture before reset

        // Apply correction to nominal state, then project covariance and reset dx
        nominal_ = Model::applyErrorState(nominal_, dx_);
        cov_     = Model::projectCovariance(cov_, dx_);
        dx_.setZero();

        RCLCPP_DEBUG(logger_,
                     "[ESKF] update   ObsDim=%d  |innov|=%.4f  |dx|=%.4f",
                     ObsDim,
                     static_cast<double>(innovation.norm()),
                     static_cast<double>(dx_norm));
        return true;
    }

    // ---- Accessors --------------------------------------------------------
    [[nodiscard]] const NavStateT& getState()       const { return nominal_; }
    [[nodiscard]] const PMat&      getCovariance()  const { return cov_; }

    void setState(const NavStateT& state)  { nominal_ = state; }
    void setState(NavStateT&& state)       { nominal_ = std::move(state); }
    void setCovariance(const PMat& P)      { cov_ = P; }

    [[nodiscard]] const typename Model::Options& getModelOptions() const { return model_opts_; }
    void setModelOptions(const typename Model::Options& opts)            { model_opts_ = opts; }

private:
    NavStateT               nominal_{};               ///< nominal (best-estimate) state
    ErrorVec                dx_  = ErrorVec::Zero();  ///< error state (zero between resets)
    PMat                    cov_;                     ///< error-state covariance  P
    typename Model::Options model_opts_;              ///< process-noise parameters
    rclcpp::Logger          logger_;
};

// ---- Convenience aliases --------------------------------------------------
using ESKFd = ESKF<NavigationModel<double>>;
using ESKFf = ESKF<NavigationModel<float>>;

}  // namespace finenav

