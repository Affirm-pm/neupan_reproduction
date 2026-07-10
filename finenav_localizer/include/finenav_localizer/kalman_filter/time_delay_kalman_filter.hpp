// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_localizer/kalman_filter/kalman_filter.hpp"

#include <iostream>
#include <utility>

namespace finenav::localizer {

/**
 * @brief TimeDelayKalmanFilter  —  KF with state augmentation for delayed observations
 * @details
 *      Extends KalmanFilter by maintaining an augmented state vector that stacks the
 *      current state together with a bounded history of past states:
 * @code
 *          x_ex = [ x_k | x_{k-1} | … | x_{k-d+1} ]   (dim_x · max_delay_step)
 * @endcode
 *      The augmented transition / process-noise matrices have the structure:
 * @code
 *          A_ex = [ A   0   0 … ]      Q_ex = [ Q   0   0 … ]
 *                 [ I   0   0 … ]             [ 0   0   0 … ]
 *                 [ 0   I   0 … ]             [ 0   0   0 … ]
 *                 [ …   …   … … ]             [ …   …   … … ]
 * @endcode
 *      An observation arriving with delay_step d is associated with x_{k-d} through the
 *      measurement matrix
 * @code
 *      C_ex = [ 0 | … | C | … | 0 ]  (C at column block d).
 * @endcode
 *      The standard KF update then corrects x_{k-d} directly and x_k, x_{k-1}, … indirectly
 *      via the cross-covariance blocks.
 *
 */
class TimeDelayKalmanFilter : public KalmanFilter {
public:
    TimeDelayKalmanFilter() = default;

    /**
     * @brief Initialize every delay slot with the same state and covariance.
     * @param x initial state vector          (dim_x × 1)
     * @param P0 initial covariance matrix    (dim_x × dim_x)
     * @param max_delay_step maximum number of delay slots (≥ 1)
     * @return True if success; false if x/P0 is empty or max_delay_step < 1.
     * @note Seeding all slots with (x, P0) keeps the filter consistent from the first tick.
     */
    bool init(const Eigen::MatrixXd & x,
              const Eigen::MatrixXd & P0,
              int                    max_delay_step)
    {
        if (x.size() == 0 || P0.size() == 0 || max_delay_step < 1) { return false; }

        max_delay_step_ = max_delay_step;
        dim_x_          = static_cast<int>(x.rows());
        dim_x_ex_       = dim_x_ * max_delay_step_;

        // Fill all delay slots with the same initial state / covariance.
        x_ = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
        P_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

        for (int i = 0; i < max_delay_step_; ++i) {
            x_.block(i * dim_x_, 0,          dim_x_, 1)      = x;
            P_.block(i * dim_x_, i * dim_x_, dim_x_, dim_x_) = P0;
        }
        return true;
    }

    /**
     * @brief Read the leading (current-time) block of the augmented state.
     * @return x_k  (dim_x × 1)
     */
    [[nodiscard]] Eigen::MatrixXd getLatestX() const
    {
        return x_.block(0, 0, dim_x_, 1);
    }

    /**
     * @brief Read the leading (current-time) block of the augmented covariance.
     * @return P(0,0)  (dim_x × dim_x)
     */
    [[nodiscard]] Eigen::MatrixXd getLatestP() const
    {
        return P_.block(0, 0, dim_x_, dim_x_);
    }

    /**
     * @brief Zero out the entire augmented error-state vector.
     * @note Called after the ESKF applies the error correction to the nominal state.
     */
    void resetState()
    {
        x_.setZero();
    }

    /**
     * @brief Overwrite only the leading (0,0) covariance block.
     * @param P_new projected leading covariance  (dim_x × dim_x)
     * @note Called after the ESKF covariance projection P ← G·P·Gᵀ.
     */
    void setLeadingCovariance(const Eigen::MatrixXd & P_new)
    {
        P_.block(0, 0, dim_x_, dim_x_) = P_new;
    }

    /**
     * @brief Read the error-state vector at a specific delay slot.
     * @param slot delay slot (0 = current time, d = d ticks ago)
     * @return error-state block at the slot  (dim_x × 1)
     */
    [[nodiscard]] Eigen::MatrixXd getXAtSlot(int slot) const
    {
        return x_.block(slot * dim_x_, 0, dim_x_, 1);
    }

    /**
     * @brief Read the covariance block P(d,d) at a specific delay slot.
     * @param slot delay slot (0 = current time, d = d ticks ago)
     * @return covariance of the error-state at that slot  (dim_x × dim_x)
     * @note Clamps to the last valid slot when slot ≥ max_delay_step.
     */
    [[nodiscard]] Eigen::MatrixXd getPAtSlot(int slot) const
    {
        const int s = std::min(slot, max_delay_step_ - 1);
        return P_.block(s * dim_x_, s * dim_x_, dim_x_, dim_x_);
    }

    /**
     * @brief Slide all delay slots one step forward in time, then propagate the covariance.
     * @details
     * State slide (the oldest slot x_{k-d+1} is discarded):
     * @code
     *         x_ex ← [ x_next | x_k | x_{k-1} | … | x_{k-d+2} ]
     * @endcode
     *
     * Structured covariance propagation (block notation [i,j]):
     * @code
     *         P(0,0) ← A·P(0,0)·Aᵀ + Q
     *         P(0,j) ← A·P(0,j-1)      for j ≥ 1
     *         P(i,0) ← P(i-1,0)·Aᵀ    for i ≥ 1
     *         P(i,j) ← P(i-1,j-1)      for i,j ≥ 1
     * @endcode
     * @param x_next next current-time state
     * @param A error-state transition Jacobian    (dim_x × dim_x)
     * @param Q process-noise covariance           (dim_x × dim_x)
     */
    bool predictWithDelay(const Eigen::MatrixXd & x_next,
                          const Eigen::MatrixXd & A,
                          const Eigen::MatrixXd & Q)
    {
        const int d_dim_x = dim_x_ex_ - dim_x_;  // (max_delay_step - 1) * dim_x

        // ── Slide state vector ──────────────────────────────────────────────
        // x_tmp = [x_next; x_[0 .. d_dim_x-1]]
        Eigen::MatrixXd x_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
        x_tmp.block(0,      0, dim_x_,  1) = x_next;
        x_tmp.block(dim_x_, 0, d_dim_x, 1) = x_.block(0, 0, d_dim_x, 1);
        x_ = std::move(x_tmp);

        // ── Update extended covariance ─────────────────────────────────────
        //
        //  Symbolically (using block notation [i,j]):
        //    P_tmp[0,0] = A * P[0,0] * A' + Q
        //    P_tmp[0, 1..] = A * P[0, 0..d-2]    (first row of new cross-cov)
        //    P_tmp[1.., 0] = P[0..d-2, 0] * A'   (first col of new cross-cov)
        //    P_tmp[1.., 1..] = P[0..d-2, 0..d-2] (inner block = old leading sub)
        //
        Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

        P_tmp.block(0,      0,      dim_x_,  dim_x_)  =
            A * P_.block(0, 0, dim_x_, dim_x_) * A.transpose() + Q;

        P_tmp.block(0,      dim_x_, dim_x_,  d_dim_x) =
            A * P_.block(0, 0, dim_x_, d_dim_x);

        P_tmp.block(dim_x_, 0,      d_dim_x, dim_x_)  =
            P_.block(0, 0, d_dim_x, dim_x_) * A.transpose();

        P_tmp.block(dim_x_, dim_x_, d_dim_x, d_dim_x) =
            P_.block(0, 0, d_dim_x, d_dim_x);

        P_ = std::move(P_tmp);
        return true;
    }

    /**
     * @brief Update with a measurement whose observation function spans two state epochs.
     * @details
     *     Builds the augmented C_ex with non-zero blocks at both slots, then calls the
     *     standard KF update.  When slot_a == slot_b the two blocks are summed at the same
     *     column (the correct degenerate case).
     * @param y measurement vector                 (dim_y × 1)
     * @param C_a measurement Jacobian for slot_a  (dim_y × dim_x)
     * @param slot_a delay slot for C_a (e.g. the "to"   epoch; 0 = current tick)
     * @param C_b measurement Jacobian for slot_b  (dim_y × dim_x)
     * @param slot_b delay slot for C_b (e.g. the "from" epoch)
     * @param R measurement-noise covariance       (dim_y × dim_y)
     * @return false when either slot ≥ max_delay_step.
     * @todo 可能需要扔掉
     */
    bool updateWithTwoSlots(const Eigen::MatrixXd & y,
                            const Eigen::MatrixXd & C_a,
                            int                    slot_a,
                            const Eigen::MatrixXd & C_b,
                            int                    slot_b,
                            const Eigen::MatrixXd & R)
    {
        if (slot_a >= max_delay_step_ || slot_b >= max_delay_step_) {
            std::cerr << "[TimeDelayKF] updateWithTwoSlots: slot " << slot_a
                      << " or " << slot_b
                      << " >= max_delay_step=" << max_delay_step_ << " — ignoring.\n";
            return false;
        }
        const int dim_y = static_cast<int>(y.rows());
        Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
        C_ex.block(0, dim_x_ * slot_a, dim_y, dim_x_) += C_a;
        C_ex.block(0, dim_x_ * slot_b, dim_y, dim_x_) += C_b;
        return update(y, C_ex * x_, C_ex, R);
    }

    /**
     * @brief Update with a measurement captured delay_step ticks ago.
     * @details
     *     Builds the augmented measurement matrix C_ex = [ 0 | … | C | … | 0 ] with C at
     *     column block delay_step, so the standard KF update simultaneously corrects
     *     x_{k-delay_step} directly and x_k, x_{k-1}, … indirectly via cross-covariance.
     * @param y measurement vector           (dim_y × 1)
     * @param C measurement Jacobian         (dim_y × dim_x)
     * @param R measurement-noise covariance (dim_y × dim_y)
     * @param delay_step 0 = no delay (current tick), 1 = one tick old, …
     * @return false when delay_step ≥ max_delay_step.
     */
    bool updateWithDelay(const Eigen::MatrixXd & y,
                         const Eigen::MatrixXd & C,
                         const Eigen::MatrixXd & R,
                         int                    delay_step)
    {
        if (delay_step >= max_delay_step_) {
            std::cerr << "[TimeDelayKF] delay_step=" << delay_step
                      << " >= max_delay_step=" << max_delay_step_
                      << " — ignoring update.\n";
            return false;
        }

        const int dim_y = static_cast<int>(y.rows());

        // Build expanded C: C_ex[:, delay_step*dim_x : (delay_step+1)*dim_x] = C
        Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
        C_ex.block(0, dim_x_ * delay_step, dim_y, dim_x_) = C;

        // y_pred = C_ex * x_ (note: in ESKF x_ is always ≈ 0)
        return update(y, C_ex * x_, C_ex, R);
    }

    /// Introspection
    [[nodiscard]] int maxDelayStep() const { return max_delay_step_; }
    [[nodiscard]] int dimX()         const { return dim_x_; }
    [[nodiscard]] int dimXEx()       const { return dim_x_ex_; }

private:
    int max_delay_step_ = 1;
    int dim_x_          = 0;
    int dim_x_ex_       = 0;
};

}  // namespace finenav::localizer
