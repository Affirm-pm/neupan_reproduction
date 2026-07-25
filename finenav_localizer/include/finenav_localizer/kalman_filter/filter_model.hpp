// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

namespace finenav::localizer {

/// Dimension of the error-state vector: [δp(3) | δθ(3) | δv(3) | δω(3)].
inline constexpr int kNavStateDim = 12;

/**
 * @brief NavState<S>  —  nominal state for the [p, R, v, ω] navigation model.
 * @details
 *      All quantities are in the world frame unless noted; ω is the body-frame angular
 *      rate (same convention as an IMU).
 * @tparam S scalar type (float / double).
 */
template <typename S = double>
struct NavState {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Ensure byte alignment when NavState in stored in heap containers

    using SO3T  = Sophus::SO3<S>;
    using Vec3T = Eigen::Matrix<S, 3, 1>;

    Vec3T p     = Vec3T::Zero();   ///< position  (world frame)
    SO3T  R;                       ///< rotation  world ← body  (identity by default)
    Vec3T v     = Vec3T::Zero();   ///< linear velocity  (world frame)
    Vec3T omega = Vec3T::Zero();   ///< angular velocity (body frame)

    NavState() = default;
    NavState(Vec3T position, SO3T rotation, Vec3T velocity, Vec3T ang_vel)
        : p(std::move(position))
        , R(std::move(rotation))
        , v(std::move(velocity))
        , omega(std::move(ang_vel))
    {}
};

using NavStateF = NavState<float>;
using NavStateD = NavState<double>;

/**
 * @brief NavigationModel<S>  —  constant-velocity dead-reckoning model.
 * @details
 *      Nominal-state propagation over dt:
 * @code
 *      p ← p + v·dt      R ← R · Exp(ω·dt)      v ← v      ω ← ω
 * @endcode
 *      Error-state layout (right-perturbation, 12-dim): [ δp(3) | δθ(3) | δv(3) | δω(3) ].
 *      Exposes only static methods, acting as a pure "model descriptor" consumed by ESKF<Model>.
 * @tparam S scalar type (float / double).
 */
template <typename S = double>
class NavigationModel {
public:
    /// Types exported for ESKF<Model>.
    using ScalarT   = S;
    using SO3T      = Sophus::SO3<S>;
    using Vec3T     = Eigen::Matrix<S, 3, 1>;
    using Mat3T     = Eigen::Matrix<S, 3, 3>;
    using NavStateT = NavState<S>;
    using ErrorVec  = Eigen::Matrix<S, kNavStateDim, 1>;
    using FMat      = Eigen::Matrix<S, kNavStateDim, kNavStateDim>;  ///< state-transition Jacobian
    using QMat      = Eigen::Matrix<S, kNavStateDim, kNavStateDim>;  ///< process-noise covariance
    using PMat      = Eigen::Matrix<S, kNavStateDim, kNavStateDim>;  ///< state covariance

    /**
     * @brief Process-noise parameters (constant-velocity model: noise enters through v and ω).
     */
    struct Options {
        S pos_noise_std   = static_cast<S>(0.001); ///< unmodeled position noise σ        [m/√s]
        S rot_noise_std   = static_cast<S>(0.001); ///< unmodeled rotation noise σ        [rad/√s]
        S vel_noise_std   = static_cast<S>(0.10);  ///< lin-velocity process noise σ      [m/s/√s]
        S omega_noise_std = static_cast<S>(0.10);  ///< ang-velocity process noise σ      [rad/s/√s]
    };

    /**
     * @brief Propagate the nominal state forward by dt (constant velocity / angular velocity).
     * @param s current nominal state.
     * @param dt time step [s].
     * @return propagated nominal state.
     */
    [[nodiscard]]
    static NavStateT predictNominal(const NavStateT& s, S dt) {
        NavStateT ns;
        ns.p     = s.p + s.v * dt;
        ns.R     = s.R * SO3T::exp(s.omega * dt);
        ns.v     = s.v;
        ns.omega = s.omega;
        return ns;
    }

    /**
     * @brief Error-state transition Jacobian F (12×12).
     * @details
     *      First-order perturbation of the nominal dynamics:
     * @code
     *      δp' = δp + δv·dt
     *      δθ' ≈ Exp(−ω·dt)·δθ + I·dt·δω     (Jr(ω·dt) ≈ I for small ω·dt)
     *      δv' = δv        δω' = δω
     * @endcode
     * @param s current nominal state.
     * @param dt time step [s].
     * @return F  (12×12).
     */
    [[nodiscard]]
    static FMat computeF(const NavStateT& s, S dt) {
        FMat F = FMat::Identity();

        // ∂δp / ∂δv
        F.template block<3, 3>(0, 6) = Mat3T::Identity() * dt;

        // ∂δθ / ∂δθ  —  Exp(−ω·dt): tangent transported by the rotation increment
        F.template block<3, 3>(3, 3) = SO3T::exp(-s.omega * dt).matrix();

        // ∂δθ / ∂δω  —  first-order: Jr(ω·dt)·dt ≈ I·dt
        F.template block<3, 3>(3, 9) = Mat3T::Identity() * dt;

        return F;
    }

    /**
     * @brief Discrete process-noise covariance Q (12×12).
     * @param opts process-noise parameters.
     * @param dt time step [s].
     * @return Q with noise on δv and δω only (constant-velocity model).
     */
    [[nodiscard]]
    static QMat buildQ(const Options& opts, S dt) {
        QMat Q = QMat::Zero();
        const S dt2 = dt * dt;

        auto diag3 = [](S sigma_sq_dt) -> Mat3T {
            return Mat3T::Identity() * sigma_sq_dt;
        };

        // All four state blocks receive independent diagonal process noise.
        // The position/rotation terms are tiny (unmodelled dynamics) but prevent
        // the corresponding covariance blocks from collapsing when the velocity
        // channels are observed with high precision (e.g. Twist ωz → P(5,5)→0).
        Q.template block<3, 3>(0, 0) = diag3(opts.pos_noise_std   * opts.pos_noise_std   * dt);
        Q.template block<3, 3>(3, 3) = diag3(opts.rot_noise_std   * opts.rot_noise_std   * dt);
        Q.template block<3, 3>(6, 6) = diag3(opts.vel_noise_std   * opts.vel_noise_std   * dt);
        Q.template block<3, 3>(9, 9) = diag3(opts.omega_noise_std * opts.omega_noise_std * dt);

        return Q;
    }

    /**
     * @brief Inject an error-state correction into the nominal state: nominal ⊕ dx.
     * @param nominal current nominal state.
     * @param dx error-state correction (12×1).
     * @return corrected nominal state.
     * @note The caller is responsible for zeroing dx afterwards.
     */
    [[nodiscard]]
    static NavStateT applyErrorState(const NavStateT& nominal, const ErrorVec& dx) {
        NavStateT corrected;
        corrected.p     = nominal.p     + dx.template segment<3>(0);
        corrected.R     = nominal.R     * SO3T::exp(dx.template segment<3>(3));
        corrected.v     = nominal.v     + dx.template segment<3>(6);
        corrected.omega = nominal.omega + dx.template segment<3>(9);
        return corrected;
    }

    /**
     * @brief ESKF covariance reset projection: P ← J·P·Jᵀ.
     * @details J = I − 0.5·[δθ]× in the (3,3) rotation block (ESKF textbook eq. 3.63).
     * @param P covariance before projection.
     * @param dx applied error-state correction (12×1).
     * @return projected covariance.
     */
    [[nodiscard]]
    static PMat projectCovariance(const PMat& P, const ErrorVec& dx) {
        FMat J = FMat::Identity();
        J.template block<3, 3>(3, 3) =
            Mat3T::Identity() - static_cast<S>(0.5) * SO3T::hat(dx.template segment<3>(3));
        return J * P * J.transpose();
    }

    /// Typed aggregate returned by every observation helper.
    template <int Dim>
    struct ObsData {
        Eigen::Matrix<S, Dim, kNavStateDim> H;           ///< observation Jacobian
        Eigen::Matrix<S, Dim, 1>            innovation;  ///< z − h(x̂)
    };

    /// Typed aggregate returned by dual-epoch observation helpers.
    template <int Dim>
    struct DualObsData {
        Eigen::Matrix<S, Dim, kNavStateDim> H_to;        ///< Jacobian w.r.t. "to"   epoch error state
        Eigen::Matrix<S, Dim, kNavStateDim> H_from;      ///< Jacobian w.r.t. "from" epoch error state
        Eigen::Matrix<S, Dim, 1>            innovation;
    };

    /**
     * @brief Observe 6-DOF pose [p_world, R_world←body].
     * @details
     * @code
     *      h_p(x) = p,   ∂h_p/∂δp = I₃
     *      h_θ(x) = δθ,  ∂h_θ/∂δθ = I₃   (linearised via log map)
     *      H (6×12) = [ I₃ 0 0 0 ; 0 I₃ 0 0 ]
     * @endcode
     * @param nominal nominal state to linearise at.
     * @param pos_obs observed position (world frame).
     * @param rot_obs observed rotation (world ← body).
     * @return H and innovation.
     */
    [[nodiscard]]
    static ObsData<6> observePose(const NavStateT& nominal,
                                  const Vec3T&     pos_obs,
                                  const SO3T&      rot_obs)
    {
        Eigen::Matrix<S, 6, kNavStateDim> H = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        H.template block<3, 3>(0, 0) = Mat3T::Identity();  // ∂z_p / ∂δp
        H.template block<3, 3>(3, 3) = Mat3T::Identity();  // ∂z_θ / ∂δθ

        Eigen::Matrix<S, 6, 1> innov;
        innov.template head<3>() = pos_obs - nominal.p;
        innov.template tail<3>() = (nominal.R.inverse() * rot_obs).log();

        return {H, innov};
    }

    /**
     * @brief Observe body-frame twist [v_body, ω_body].
     * @details
     *      State stores v in world frame, so the observation function is h_v(x) = Rᵀ·v,
     *      h_ω(x) = ω. First-order Jacobian (right-perturbation R = R̂·Exp(δθ)):
     * @code
     *      ∂h_v/∂δθ = [R̂ᵀv̂]×      ∂h_v/∂δv = R̂ᵀ      ∂h_ω/∂δω = I₃
     *      H (6×12) = [ 0 [R̂ᵀv̂]× R̂ᵀ 0 ; 0 0 0 I₃ ]
     * @endcode
     * @param nominal nominal state to linearise at.
     * @param lin_vel_obs_body observed linear velocity (body frame).
     * @param ang_vel_obs_body observed angular velocity (body frame).
     * @return H and innovation.
     */
    [[nodiscard]]
    static ObsData<6> observeVelocity(const NavStateT& nominal,
                                      const Vec3T&     lin_vel_obs_body,
                                      const Vec3T&     ang_vel_obs_body)
    {
        const Mat3T Rt         = nominal.R.inverse().matrix();  // R̂^T
        const Vec3T v_body_hat = Rt * nominal.v;                // R̂^T · v̂

        Eigen::Matrix<S, 6, kNavStateDim> H = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        H.template block<3, 3>(0, 3) = SO3T::hat(v_body_hat);  // ∂v_body / ∂δθ
        H.template block<3, 3>(0, 6) = Rt;                     // ∂v_body / ∂δv
        H.template block<3, 3>(3, 9) = Mat3T::Identity();       // ∂ω_body / ∂δω

        Eigen::Matrix<S, 6, 1> innov;
        innov.template head<3>() = lin_vel_obs_body - v_body_hat;
        innov.template tail<3>() = ang_vel_obs_body - nominal.omega;

        return {H, innov};
    }

    /**
     * @brief Observe relative pose [Δp_body, ΔR] spanning two epochs (Stochastic Cloning).
     * @details
     *      Measurement:  z_p = R_from⁻¹·(p_to − p_from),   z_R = R_from⁻¹·R_to.
     *      Predicted:    ẑ_p = R̂_from⁻¹·Δp̂  (Δp̂ = p̂_to − p̂_from),   ẑ_R = R̂_from⁻¹·R̂_to.
     * @code
     *      H_to   = [ R̂_from⁻¹   0                  0 0 ; 0   I      0 0 ]
     *      H_from = [ −R̂_from⁻¹  [R̂_from⁻¹·Δp̂]×    0 0 ; 0  −ΔR̂ᵀ   0 0 ]
     * @endcode
     * @param from_state nominal at the "from" epoch.
     * @param to_state nominal at the "to" epoch.
     * @param delta_p_body_obs measured body-frame displacement.
     * @param delta_R_obs measured relative rotation.
     * @return H_to, H_from and innovation.
     * @note R_noise (supplied by the caller) must be a DISPLACEMENT covariance (m² / rad²), not velocity.
     */
    [[nodiscard]]
    static DualObsData<6> observeRelativePose(const NavStateT& from_state,
                                              const NavStateT& to_state,
                                              const Vec3T&     delta_p_body_obs,
                                              const SO3T&      delta_R_obs)
    {
        const Mat3T Rf_inv     = from_state.R.inverse().matrix();          // R̂_from^{-1}
        const Vec3T delta_p_hat = to_state.p - from_state.p;              // Δp̂ (world frame)
        const SO3T  delta_R_hat = from_state.R.inverse() * to_state.R;    // ΔR̂

        // ── H_to ─────────────────────────────────────────────────────────────
        Eigen::Matrix<S, 6, kNavStateDim> Hto = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        Hto.template block<3, 3>(0, 0) = Rf_inv;              // ∂Δp / ∂δp_to
        Hto.template block<3, 3>(3, 3) = Mat3T::Identity();   // ∂ΔR / ∂δθ_to

        // ── H_from ───────────────────────────────────────────────────────────
        Eigen::Matrix<S, 6, kNavStateDim> Hfrom = Eigen::Matrix<S, 6, kNavStateDim>::Zero();
        Hfrom.template block<3, 3>(0, 0) = -Rf_inv;                              // ∂Δp / ∂δp_from
        Hfrom.template block<3, 3>(0, 3) = SO3T::hat(Rf_inv * delta_p_hat);      // ∂Δp / ∂δθ_from  ([R̂_from^{-1}Δp̂]×)
        Hfrom.template block<3, 3>(3, 3) = -delta_R_hat.matrix().transpose();    // ∂ΔR / ∂δθ_from  (−ΔR̂^T)

        // ── Innovation ───────────────────────────────────────────────────────
        Eigen::Matrix<S, 6, 1> innov;
        innov.template head<3>() = delta_p_body_obs - Rf_inv * delta_p_hat;
        innov.template tail<3>() = (delta_R_hat.inverse() * delta_R_obs).log();

        return {Hto, Hfrom, innov};
    }

};

using NavigationModelF = NavigationModel<float>;
using NavigationModelD = NavigationModel<double>;

}  // namespace finenav::localizer
