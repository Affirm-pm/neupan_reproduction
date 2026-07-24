// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <tuple>

#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "finenav_localizer/kalman_filter/eskf.hpp"
#include "finenav_localizer/kalman_filter/filter_model.hpp"
#include "finenav_localizer/observation_queue.hpp"
#include "finenav_localizer/localizer_params.hpp"

namespace finenav::localizer {

/// Per-sensor overrides carried alongside a pose observation in the internal queue.
struct TaggedPoseObs {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    int                  smooth{1};
    double               gate_dist{1e9};
    std::array<bool, 6>  mask{{true, true, true, true, true, true}};
    std::array<double, 6> cov_override{};  // < 0 → use from msg; ≥ 0 → override
};

/// Per-sensor overrides carried alongside a twist observation.
struct TaggedTwistObs {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    int                  smooth{1};
    double               gate_dist{1e9};
    std::array<bool, 6>  mask{{true, true, true, true, true, true}};
    std::array<double, 6> cov_override{};  // < 0 → use from msg; ≥ 0 → override
};

// Internal queues now carry tagged wrappers — per-sensor config lives with the message.
using PoseObsQueue  = AgedObservationQueue<TaggedPoseObs>;
using TwistObsQueue = AgedObservationQueue<TaggedTwistObs>;

/**
 * @brief EskfLocalizer  —  ROS-free ESKF localization engine (the C++ API layer)
 */
class EskfLocalizer {
public:
    using PMat     = NavigationModelD::PMat;
    using Vec3d    = NavigationModelD::Vec3T;
    using SO3d     = NavigationModelD::SO3T;
    using PoseMsg  = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;

    EskfLocalizer() : EskfLocalizer(finenav_localizer::Params{}) {}
    explicit EskfLocalizer(const finenav_localizer::Params & params,
                           rclcpp::Logger logger = rclcpp::get_logger("finenav.EskfLocalizer"));

    EskfLocalizer(const EskfLocalizer &)             = delete;
    EskfLocalizer & operator=(const EskfLocalizer &) = delete;

    /**
     * @brief Live-update the per-tick-read tunables Does not rebuild the estimator.
     */
    void setConfig(const finenav_localizer::Params & params);
    [[nodiscard]] const finenav_localizer::Params & getConfig() const { return params_; }

    /**
     * @brief Reset the filter to the given pose.  Clears all delay-slot history.
     */
    void setInitialPose(const PoseMsg & msg);

    /**
     * @brief Enqueue a pose / twist / relative-pose observation.
     * @details
     *      Global-params overloads (C++ API): smooth / gate / mask use defaults.
     *      Tagged overloads (ROS sensor callbacks): per-sensor overrides.
     */
    void feedObservation(const PoseMsg  & msg);
    void feedObservation(const PoseMsg  & msg, int smooth, double gate,
                         const std::array<bool,6>& mask,
                         const std::array<double,6>& cov_override = {});
    void feedObservation(const TwistMsg & msg);
    void feedObservation(const TwistMsg & msg, int smooth, double gate,
                         const std::array<bool,6>& mask,
                         const std::array<double,6>& cov_override = {});
    void feedRelativePose(const RelativePoseObs & obs);
    /// Planar constraint injection (called each tick when force_2d_mode is on).
    void applyPlanarConstraints();

    /**
     * @brief Advance the estimator by one cycle using @p now as the reference epoch.
     */
    void step(const rclcpp::Time & now);

    /// Latest state, `v` in the **body frame** (API contract).
    [[nodiscard]] NavStateD    getState()          const;
    /// Raw latest nominal state (world-frame v, no force_2d shaping) — for TF.
    [[nodiscard]] NavStateD    getRawState()       const;
    [[nodiscard]] PMat         getCovariance()     const;
    [[nodiscard]] rclcpp::Time getLastUpdateTime() const;
    [[nodiscard]] bool isInitialized() const { return is_initialized_.load(std::memory_order_acquire); }

private:
    void configure(const finenav_localizer::Params & params);

    void applyUpdate(const TaggedPoseObs   & tagged, int delay_step);
    void applyUpdate(const TaggedTwistObs  & tagged, int delay_step);
    void applyUpdate(const RelativePoseObs & obs, int delay_step_to, int delay_step_from);

    /// Mask a full 6-DOF observation (H, innov, R) down to k selected rows.
    static std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>
    maskObservation(const Eigen::Matrix<double, 6, 12>& H_full,
                    const Eigen::Matrix<double, 6,  1>& innov_full,
                    const Eigen::Matrix<double, 6,  6>& R_full,
                    const std::array<bool, 6>&           config);

    static std::tuple<Vec3d, SO3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const PoseMsg & msg);
    static std::tuple<Vec3d, Vec3d, Eigen::Matrix<double, 6, 6>>
    extractObservation(const TwistMsg & msg);

    NavStateD force2DOutput(const NavStateD & s) const;

    finenav_localizer::Params params_;                ///< latest parameter snapshot TODO: 只存储地址，不存储值

    mutable std::mutex          filter_mutex_;        ///< guards eskf_ + predict_time + init
    ESKFd                       eskf_;                ///< error-state Kalman filter
    std::optional<rclcpp::Time> last_predict_time_;   ///< dt reference

    std::unique_ptr<PoseObsQueue>         pose_queue_;      ///< pose observations
    std::unique_ptr<TwistObsQueue>        twist_queue_;     ///< twist observations
    std::unique_ptr<RelativePoseObsQueue> rel_pose_queue_;  ///< rel-pose (odometry)

    std::atomic<bool> is_initialized_{false};

    mutable std::shared_mutex cached_state_mutex_;        ///< guards the output cache below
    NavStateD    cached_state_{};
    PMat         cached_covariance_ = PMat::Identity() * 1e-2;
    rclcpp::Time cached_time_{0, 0, RCL_ROS_TIME};

    /// Predict-guard: freeze the filter when all queues are empty for too long.
    int consecutive_empty_ticks_{0};
    int max_empty_ticks_{10};          // set from params in configure()

    rclcpp::Logger logger_;
};

}  // namespace finenav::localizer
