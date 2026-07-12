// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/LU>   // MatrixBase::inverse() for dynamic matrices

namespace finenav::localizer::kf {

/**
 * @brief Standard linear KF prediction, applied in place to (x, P):
 * @code
 *   x ← x_next            (caller supplies the propagated state)
 *   P ← A·P·Aᵀ + Q
 * @endcode
 *  @param x       state vector, updated in place                    (dim_x × 1)
 *  @param P       covariance, updated in place                      (dim_x × dim_x)
 *  @param x_next  pre-computed next state (possibly non-linear model)  (dim_x × 1)
 *  @param A       state-transition Jacobian                         (dim_x × dim_x)
 *  @param Q       process-noise covariance                          (dim_x × dim_x)
 *  @return false on a dimension mismatch; (x, P) left unchanged
 */
inline bool predict(Eigen::MatrixXd&       x,
                    Eigen::MatrixXd&       P,
                    const Eigen::MatrixXd& x_next,
                    const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& Q)
{
    if (x.rows() != x_next.rows() ||
        A.cols() != P.rows()      ||
        Q.rows() != Q.cols()      ||
        A.rows() != Q.cols()) {
        return false;
    }

    x = x_next;
    P = A * P * A.transpose() + Q;

    return true;
}

/**
 * @brief Standard linear KF measurement update, applied **in place** to (x, P):
 * @code
 *   K = P·Cᵀ·(C·P·Cᵀ + R)⁻¹
 *   x ← x + K·(y − y_pred)
 *   P ← (I − K·C)·P     computed as P − K·(C·P) = P − K·PCTᵀ
 *   P ← ½·(P + Pᵀ)      force symmetry
 * @endcode
 *  @param x       state vector, updated in place        (dim_x × 1)
 *  @param P       covariance, updated in place          (dim_x × dim_x)
 *  @param y       measurement vector                    (dim_y × 1)
 *  @param y_pred  predicted observation h(x̂)            (dim_y × 1)
 *  @param C       observation Jacobian                  (dim_y × dim_x)
 *  @param R       observation-noise covariance          (dim_y × dim_y)
 *  @return false on a dimension mismatch, or if the Kalman gain is NaN/Inf; in both cases (x, P) are left unchanged
 */
inline bool update(Eigen::MatrixXd&       x,
                   Eigen::MatrixXd&       P,
                   const Eigen::MatrixXd& y,
                   const Eigen::MatrixXd& y_pred,
                   const Eigen::MatrixXd& C,
                   const Eigen::MatrixXd& R)
{
    if (P.cols() != C.cols() ||
        R.rows() != R.cols()  ||
        R.rows() != C.rows()  ||
        y.rows() != y_pred.rows() ||
        y.rows() != C.rows()) {
        return false;
    }

    const Eigen::MatrixXd PCT = P * C.transpose();
    const Eigen::MatrixXd K   = PCT * (R + C * PCT).inverse();

    if (K.array().isNaN().any() || K.array().isInf().any()) {
        return false;
    }

    x = x + K * (y - y_pred);

    P.noalias() -= K * PCT.transpose();          // P = (I − KC)·P = P − K·PCTᵀ
    P = (0.5 * (P + P.transpose())).eval();       // force symmetry

    return true;
}

}  // namespace finenav::localizer::kf
