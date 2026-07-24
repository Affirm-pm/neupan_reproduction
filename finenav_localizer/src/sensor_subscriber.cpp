// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/sensor_subscriber.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>

#include "finenav_localizer/eskf_localizer.hpp"

namespace finenav::localizer {

SensorSubscriber::SensorSubscriber(rclcpp::Node & node, EskfLocalizer & localizer)
    : node_(node)
    , localizer_(localizer)
    , logger_(node_.get_logger().get_child("SensorSubscriber"))
{}

void SensorSubscriber::discoverAndSubscribe()
{
    discoverPoseSensors();
    discoverTwistSensors();
    discoverOdomSensors();

    const std::string init_topic =
        node_.declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    if (!init_topic.empty()) {
        initial_pose_sub_ = node_.create_subscription<PoseMsg>(
            init_topic, rclcpp::SystemDefaultsQoS(),
            [this](PoseMsg::ConstSharedPtr msg) {
                localizer_.setInitialPose(*msg);
                initial_pose_sub_.reset();  // one-shot
            });
    }
}

bool SensorSubscriber::hasSensors() const
{
    return !pose_sensors_.empty() || !twist_sensors_.empty() || !odom_sensors_.empty();
}

// ── Pose sensors ──────────────────────────────────────────────────────────

void SensorSubscriber::discoverPoseSensors()
{
    for (int i = 0; ; ++i) {
        const std::string key   = fmt::format("pose{}", i);
        const std::string topic = node_.declare_parameter<std::string>(key, "");
        if (topic.empty()) break;

        PoseSensor s;
        s.topic = topic;

        const std::string cfg_key = fmt::format("pose{}_config", i);
        const auto cfg = node_.declare_parameter<std::vector<bool>>(
            cfg_key, std::vector<bool>(6, true));
        for (std::size_t j = 0; j < 6 && j < cfg.size(); ++j) s.mask[j] = cfg[j];

        s.smooth    = node_.declare_parameter<int>(fmt::format("pose{}_smoothing_steps", i), 1);
        s.gate_dist = node_.declare_parameter<double>(fmt::format("pose{}_gate_dist", i), 1e9);

        const auto cov_ov = node_.declare_parameter<std::vector<double>>(
            fmt::format("pose{}_covariance_override", i), std::vector<double>(6, -1.0));
        for (std::size_t j = 0; j < 6 && j < cov_ov.size(); ++j) s.cov_override[j] = cov_ov[j];

        s.sub = node_.create_subscription<PoseMsg>(
            topic, rclcpp::SystemDefaultsQoS(),
            [this, idx = pose_sensors_.size()](PoseMsg::ConstSharedPtr msg) {
                poseCallback(std::move(msg), idx);
            });
        pose_sensors_.push_back(std::move(s));
    }
}

void SensorSubscriber::poseCallback(PoseMsg::ConstSharedPtr msg, std::size_t idx)
{
    const auto & s = pose_sensors_.at(idx);
    localizer_.feedObservation(*msg, s.smooth, s.gate_dist, s.mask, s.cov_override);
}

// ── Twist sensors ─────────────────────────────────────────────────────────

void SensorSubscriber::discoverTwistSensors()
{
    for (int i = 0; ; ++i) {
        const std::string key   = fmt::format("twist{}", i);
        const std::string topic = node_.declare_parameter<std::string>(key, "");
        if (topic.empty()) break;

        TwistSensor s;
        s.topic = topic;

        const std::string cfg_key = fmt::format("twist{}_config", i);
        const auto cfg = node_.declare_parameter<std::vector<bool>>(
            cfg_key, std::vector<bool>(6, true));
        for (std::size_t j = 0; j < 6 && j < cfg.size(); ++j) s.mask[j] = cfg[j];

        s.smooth    = node_.declare_parameter<int>(fmt::format("twist{}_smoothing_steps", i), 1);
        s.gate_dist = node_.declare_parameter<double>(fmt::format("twist{}_gate_dist", i), 1e9);

        const auto cov_ov = node_.declare_parameter<std::vector<double>>(
            fmt::format("twist{}_covariance_override", i), std::vector<double>(6, -1.0));
        for (std::size_t j = 0; j < 6 && j < cov_ov.size(); ++j) s.cov_override[j] = cov_ov[j];

        s.sub = node_.create_subscription<TwistMsg>(
            topic, rclcpp::SystemDefaultsQoS(),
            [this, idx = twist_sensors_.size()](TwistMsg::ConstSharedPtr msg) {
                twistCallback(std::move(msg), idx);
            });
        twist_sensors_.push_back(std::move(s));
    }
}

void SensorSubscriber::twistCallback(TwistMsg::ConstSharedPtr msg, std::size_t idx)
{
    const auto & s = twist_sensors_.at(idx);
    localizer_.feedObservation(*msg, s.smooth, s.gate_dist, s.mask, s.cov_override);
}

// ── Odometry sensors ──────────────────────────────────────────────────────

void SensorSubscriber::discoverOdomSensors()
{
    for (int i = 0; ; ++i) {
        const std::string key   = fmt::format("odom{}", i);
        const std::string topic = node_.declare_parameter<std::string>(key, "");
        if (topic.empty()) break;

        OdomSensor s;
        s.topic = topic;

        s.use_relative = (node_.declare_parameter<std::string>(
                              fmt::format("odom{}_mode", i), "relative_pose") != "twist");

        const std::string cfg_key = fmt::format("odom{}_config", i);
        const auto cfg = node_.declare_parameter<std::vector<bool>>(
            cfg_key, std::vector<bool>(6, true));
        for (std::size_t j = 0; j < 6 && j < cfg.size(); ++j) s.mask[j] = cfg[j];

        s.smooth    = node_.declare_parameter<int>(fmt::format("odom{}_smoothing_steps", i), 1);
        s.gate_dist = node_.declare_parameter<double>(fmt::format("odom{}_gate_dist", i), 1e9);

        const auto cov_ov = node_.declare_parameter<std::vector<double>>(
            fmt::format("odom{}_covariance_override", i), std::vector<double>(6, -1.0));
        for (std::size_t j = 0; j < 6 && j < cov_ov.size(); ++j) s.cov_override[j] = cov_ov[j];

        const auto abs_pose_cfg = node_.declare_parameter<std::vector<bool>>(
            fmt::format("odom{}_extract_absolute_pose", i), std::vector<bool>(6, false));
        for (std::size_t j = 0; j < 6 && j < abs_pose_cfg.size(); ++j)
            s.extract_absolute_pose[j] = abs_pose_cfg[j];

        s.sub = node_.create_subscription<OdomMsg>(
            topic, rclcpp::SystemDefaultsQoS(),
            [this, idx = odom_sensors_.size()](OdomMsg::ConstSharedPtr msg) {
                odomCallback(std::move(msg), idx);
            });
        odom_sensors_.push_back(std::move(s));
    }
}

void SensorSubscriber::odomCallback(OdomMsg::ConstSharedPtr msg, std::size_t idx)
{
    auto & s = odom_sensors_.at(idx);

    if (s.use_relative) {
        // RelPose mode: compute body-frame displacement between consecutive frames.
        if (!s.prev_msg.has_value()) {
            s.prev_msg = *msg;
            return;
        }

        const auto & prev = *s.prev_msg;
        const auto & curr = *msg;

        const Eigen::Quaterniond q_prev{
            prev.pose.pose.orientation.w, prev.pose.pose.orientation.x,
            prev.pose.pose.orientation.y, prev.pose.pose.orientation.z};
        const Sophus::SO3d R_prev{q_prev.normalized().toRotationMatrix()};

        const Eigen::Quaterniond q_curr{
            curr.pose.pose.orientation.w, curr.pose.pose.orientation.x,
            curr.pose.pose.orientation.y, curr.pose.pose.orientation.z};
        const Sophus::SO3d R_curr{q_curr.normalized().toRotationMatrix()};

        const Eigen::Vector3d p_prev{prev.pose.pose.position.x,
                                     prev.pose.pose.position.y,
                                     prev.pose.pose.position.z};
        const Eigen::Vector3d p_curr{curr.pose.pose.position.x,
                                     curr.pose.pose.position.y,
                                     curr.pose.pose.position.z};

        RelativePoseObs rel;
        rel.smooth     = s.smooth;
        rel.gate_dist  = s.gate_dist;
        rel.delta_p_body_obs = R_prev.inverse() * (p_curr - p_prev);
        rel.delta_R_obs      = R_prev.inverse() * R_curr;

        // Combine pose covariances (upper-triangular 6×6 subset)
        rel.R_noise.setZero();
        for (int r = 0; r < 6; ++r)
            for (int c = r; c < 6; ++c) {
                const double v = prev.pose.covariance[static_cast<std::size_t>(r * 6 + c)]
                               + curr.pose.covariance[static_cast<std::size_t>(r * 6 + c)];
                rel.R_noise(r, c) = v;
                rel.R_noise(c, r) = v;
            }

        // Apply covariance overrides to the combined R_noise.
        for (int i = 0; i < 6; ++i) {
            if (s.cov_override[static_cast<std::size_t>(i)] >= 0.0) {
                rel.R_noise(i, i) = s.cov_override[static_cast<std::size_t>(i)];
            }
        }

        rel.stamp_to   = rclcpp::Time(curr.header.stamp);
        rel.stamp_from = rclcpp::Time(prev.header.stamp);

        s.prev_msg = *msg;
        localizer_.feedRelativePose(rel);
    } else {
        // Twist mode: extract body-frame velocity from odometry twist field.
        TwistMsg twist;
        twist.header           = msg->header;
        twist.twist.twist      = msg->twist.twist;
        twist.twist.covariance = msg->twist.covariance;
        localizer_.feedObservation(twist, s.smooth, s.gate_dist, s.mask, s.cov_override);
    }

    // ── Advanced: also extract absolute pose from odom.pose.pose ──────────
    if (std::any_of(s.extract_absolute_pose.begin(), s.extract_absolute_pose.end(),
                    [](bool v) { return v; })) {
        PoseMsg abs_pose;
        abs_pose.header          = msg->header;
        abs_pose.pose.pose       = msg->pose.pose;
        abs_pose.pose.covariance = msg->pose.covariance;
        localizer_.feedObservation(abs_pose, s.smooth, s.gate_dist,
                                   s.extract_absolute_pose, s.cov_override);
    }
}

}  // namespace finenav::localizer
