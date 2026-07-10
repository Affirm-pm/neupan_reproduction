// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>

namespace finenav::localizer {


/**
 *  @brief KalmanFilter  —  standard linear discrete-time KF
 *  @code
 *      Prediction:  x' = x_next,   P' = A·P·Aᵀ + Q
 *      Update:      K  = P·Cᵀ·(C·P·Cᵀ + R)⁻¹
 *                   x' = x + K·(y − y_pred)
 *                   P' = (I − K·C)·P·(I − K·C)ᵀ + K·R·Kᵀ   (Joseph form)
 *  @endcode
 */
class KalmanFilter {
public:
    KalmanFilter()          = default;
    virtual ~KalmanFilter() = default;


    /**
     * @brief Set initial state, x₀ and P₀
     * @param x initial state
     * @param P0 initial covariance matrix
     * @return True if success
     */
    bool init(const Eigen::MatrixXd & x, const Eigen::MatrixXd & P0)
    {
        if (x.size() == 0 || P0.size() == 0) { return false; }
        x_ = x;
        P_ = P0;
        return true;
    }

    /**
     * @brief Predict step of the Kalman filter.
     * @param x_next Pre-computed next state
     * @param A State-transition Jacobian  (dim_x × dim_x)
     * @param Q Process-noise covariance   (dim_x × dim_x)
     * @return True if success
     * @note Caller's responsibility to compute next step state with system model.
     */
    bool predict(const Eigen::MatrixXd & x_next,
                 const Eigen::MatrixXd & A,
                 const Eigen::MatrixXd & Q)
    {
        if (x_.rows() != x_next.rows() ||
            A.cols()  != P_.rows()     ||
            Q.rows()  != Q.cols()      ||
            A.rows()  != Q.cols()) {
            return false;
        }
        x_ = x_next;
        P_ = A * P_ * A.transpose() + Q;
        return true;
    }

    /**
     * @brief Update step of the Kalman Filter.
     * @param y measurement vector                 (dim_y × 1)
     * @param y_pred predicted observation h(x̂)    (dim_y × 1)
     * @param C observation Jacobian               (dim_y × dim_x)
     * @param R observation-noise covariance       (dim_y × dim_y)
     * @return True if success
     * @note Caller's responsibility to compute predicted observation with observation model.
     * @note Uses the Joseph form so the updated covariance stays symmetric and positive-definite.
     */
    bool update(const Eigen::MatrixXd & y,
                const Eigen::MatrixXd & y_pred,
                const Eigen::MatrixXd & C,
                const Eigen::MatrixXd & R)
    {
        if (P_.cols() != C.cols() ||
            R.rows() != R.cols()  ||
            R.rows() != C.rows()  ||
            y.rows() != y_pred.rows() ||
            y.rows() != C.rows()) {
            return false;
        }

        const Eigen::MatrixXd PCT = P_ * C.transpose();
        const Eigen::MatrixXd K   = PCT * (R + C * PCT).inverse();

        if (K.array().isNaN().any() || K.array().isInf().any()) {
            return false;
        }

        x_ = x_ + K * (y - y_pred);

        // Joseph form: P = (I-KC)P(I-KC)' + KRK'
        const Eigen::MatrixXd I_KC =
            Eigen::MatrixXd::Identity(P_.rows(), P_.cols()) - K * C;
        P_ = I_KC * P_ * I_KC.transpose() + K * R * K.transpose();

        return true;
    }

    /**
     * @brief Update step of the Kalman Filter using linear observation model y_pred = C·x̂
     * @param y measurement vector           (dim_y × 1)
     * @param C observation Jacobian         (dim_y × dim_x)
     * @param R observation-noise covariance (dim_y × dim_y)
     * @return True if success
     */
    bool update(const Eigen::MatrixXd & y,
                const Eigen::MatrixXd & C,
                const Eigen::MatrixXd & R)
    {
        if (C.cols() != x_.rows()) { return false; }
        return update(y, C * x_, C, R);
    }

    /// Accessors
    [[nodiscard]] Eigen::MatrixXd getX() const { return x_; }
    [[nodiscard]] Eigen::MatrixXd getP() const { return P_; }
    [[nodiscard]] double getXelement(int i) const { return x_(i); }

protected:
    Eigen::MatrixXd x_;   ///< state vector  (dim_x × 1)
    Eigen::MatrixXd P_;   ///< covariance    (dim_x × dim_x)
};

}  // namespace finenav::localizer

