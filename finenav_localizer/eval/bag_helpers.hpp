// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Bag-replay utilities for the eval framework (Scenario 4).
//
// Reads a rosbag2 file via rosbag2_cpp::Reader (no ROS runtime / DDS / timer),
// deserialises the topics into localizer observation messages, and drives the
// EskfLocalizer engine deterministically on a virtual clock.  The output is the
// standard eval CSV, reusing the existing plot.py for visualisation.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <numeric>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rcutils/logging.h>
#include <rclcpp/serialized_message.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <Eigen/Geometry>

#include "eval_config.hpp"
#include "finenav_localizer/eskf_localizer.hpp"

namespace fn = finenav::localizer;

// ============================================================================
//  BagMessage — one entry from the bag, tagged by role
// ============================================================================
enum class BagMsgRole { Pose, Odom, Other };

struct BagMessage {
    rclcpp::Time                    stamp;
    BagMsgRole                      role;
    fn::EskfLocalizer::PoseMsg      pose_msg;   // role == Pose
    nav_msgs::msg::Odometry         odom_msg;   // role == Odom
};

// ============================================================================
//  GtSample — ground-truth pose at one timestamp
// ============================================================================
struct GtSample {
    rclcpp::Time stamp;
    double x{}, y{}, yaw{};
};

// ── Helpers ────────────────────────────────────────────────────────────────────

/// Compute map→target = map→odom ⊕ odom→target from two transforms.
inline bool composeMapToTarget(
    const geometry_msgs::msg::TransformStamped & T_map_odom,
    const geometry_msgs::msg::TransformStamped & T_odom_target,
    double & x, double & y, double & yaw)
{
    const auto & A = T_map_odom.transform;
    const auto & B = T_odom_target.transform;

    const Eigen::Quaterniond qA(A.rotation.w, A.rotation.x, A.rotation.y, A.rotation.z);
    const Eigen::Quaterniond qB(B.rotation.w, B.rotation.x, B.rotation.y, B.rotation.z);
    const Eigen::Quaterniond q_res = qA * qB;

    const Eigen::Vector3d pA(A.translation.x, A.translation.y, A.translation.z);
    const Eigen::Vector3d pB(B.translation.x, B.translation.y, B.translation.z);
    const Eigen::Vector3d p_res = pA + qA * pB;

    x   = p_res.x();
    y   = p_res.y();
    yaw = std::atan2(2.0 * (q_res.w() * q_res.z() + q_res.x() * q_res.y()),
                     1.0 - 2.0 * (q_res.y() * q_res.y() + q_res.z() * q_res.z()));
    return true;
}

/// Build a PoseWithCovarianceStamped from a PoseStamped + diagonal covariance.
inline fn::EskfLocalizer::PoseMsg poseStampedToPoseWithCov(
    const std::shared_ptr<geometry_msgs::msg::PoseStamped> & ps,
    double var_xy, double var_yaw)
{
    fn::EskfLocalizer::PoseMsg m;
    m.header          = ps->header;
    m.pose.pose       = ps->pose;
    m.pose.covariance.fill(0.0);
    m.pose.covariance[0]  = var_xy;
    m.pose.covariance[7]  = var_xy;
    m.pose.covariance[14] = 1e-4;
    m.pose.covariance[21] = 1e-4;
    m.pose.covariance[28] = 1e-4;
    m.pose.covariance[35] = var_yaw;
    return m;
}

/// Build a TwistWithCovarianceStamped from an Odometry message (body-frame twist + covariance).
inline fn::EskfLocalizer::TwistMsg odomToTwist(const nav_msgs::msg::Odometry & odom,
                                               double var_v, double var_w)
{
    fn::EskfLocalizer::TwistMsg m;
    m.header          = odom.header;
    m.twist.twist     = odom.twist.twist;
    m.twist.covariance.fill(0.0);
    m.twist.covariance[0]  = var_v;    // vx
    m.twist.covariance[7]  = 1e-4;     // vy (unused)
    m.twist.covariance[14] = 1e-4;     // vz (unused)
    m.twist.covariance[21] = 1e-4;     // ωx (unused)
    m.twist.covariance[28] = 1e-4;     // ωy (unused)
    m.twist.covariance[35] = var_w;    // ωz
    return m;
}

/// Build a RelativePoseObs from two consecutive Odometry frames (body-frame displacement).
inline fn::RelativePoseObs odomPairToRelPose(const nav_msgs::msg::Odometry & prev,
                                             const nav_msgs::msg::Odometry & curr,
                                             double var_dp, double var_dR)
{
    // R_prev: rotation from world to body at the previous frame.
    const Eigen::Quaterniond q_prev(
        prev.pose.pose.orientation.w, prev.pose.pose.orientation.x,
        prev.pose.pose.orientation.y, prev.pose.pose.orientation.z);
    const Sophus::SO3d R_prev(q_prev.normalized().toRotationMatrix());

    const Eigen::Vector3d p_prev(prev.pose.pose.position.x,
                                 prev.pose.pose.position.y,
                                 prev.pose.pose.position.z);
    const Eigen::Vector3d p_curr(curr.pose.pose.position.x,
                                 curr.pose.pose.position.y,
                                 curr.pose.pose.position.z);
    const Eigen::Quaterniond q_curr(
        curr.pose.pose.orientation.w, curr.pose.pose.orientation.x,
        curr.pose.pose.orientation.y, curr.pose.pose.orientation.z);
    const Sophus::SO3d R_curr(q_curr.normalized().toRotationMatrix());

    fn::RelativePoseObs obs;
    obs.delta_p_body_obs = R_prev.inverse() * (p_curr - p_prev);         // body-frame Δp
    obs.delta_R_obs      = R_prev.inverse() * R_curr;                     // relative rotation
    obs.R_noise.setZero();
    obs.R_noise.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * var_dp;
    obs.R_noise.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * var_dR;
    obs.stamp_to   = rclcpp::Time(curr.header.stamp);
    obs.stamp_from = rclcpp::Time(prev.header.stamp);
    return obs;
}

// ============================================================================
//  readBagMessages + ground truth index
// ============================================================================

/// Open a rosbag2 file and read all matching messages into a time-sorted vector.
/// Also populates a separate ground-truth index from /tf (map→base_link).
inline std::vector<BagMessage> readBagMessages(
    const std::string             & bag_path,
    const std::string             & gt_frame,
    std::vector<GtSample>         & gt_index)
{
    std::vector<BagMessage> all;

    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = bag_path;
    storage_opts.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_opts;
    converter_opts.input_serialization_format  = "cdr";
    converter_opts.output_serialization_format = "cdr";

    rosbag2_cpp::Reader reader;
    reader.open(storage_opts, converter_opts);

    // Deserializers (created once).
    rclcpp::Serialization<geometry_msgs::msg::PoseStamped>   pose_deser;
    rclcpp::Serialization<nav_msgs::msg::Odometry>           odom_deser;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage>          tf_deser;

    // Detect topic types from bag metadata.
    std::string pose_topic, odom_topic, tf_topic;
    for (const auto & tm : reader.get_metadata().topics_with_message_count) {
        const auto & t = tm.topic_metadata;
        if (t.type == "geometry_msgs/msg/PoseStamped")       { pose_topic = t.name; }
        if (t.type == "nav_msgs/msg/Odometry")               { odom_topic = t.name; }
        if (t.type == "tf2_msgs/msg/TFMessage" && t.name != "/tf_static") { tf_topic = t.name; }
    }

    RCLCPP_INFO(rclcpp::get_logger("bag_helpers"),
        "[bag_helpers] pose_topic='%s'  odom_topic='%s'  tf_topic='%s'",
        pose_topic.c_str(), odom_topic.c_str(), tf_topic.c_str());

    // Collect stamp-delta statistics.
    std::vector<double> pose_deltas, odom_deltas;

    // Accumulate the latest map→odom and odom→gt_frame transforms for chaining.
    geometry_msgs::msg::TransformStamped latest_map_odom;
    geometry_msgs::msg::TransformStamped latest_odom_target;
    bool have_map_odom    = false;
    bool have_odom_target = false;

    while (reader.has_next()) {
        auto serialised = reader.read_next();
        const rclcpp::Time stamp(serialised->time_stamp, RCL_ROS_TIME);

        // ── PoseStamped ──────────────────────────────────────────────────────
        if (serialised->topic_name == pose_topic) {
            rclcpp::SerializedMessage ser_msg(*serialised->serialized_data);
            auto ps = std::make_shared<geometry_msgs::msg::PoseStamped>();
            pose_deser.deserialize_message(&ser_msg, ps.get());

            pose_deltas.push_back(rclcpp::Time(ps->header.stamp).seconds() - stamp.seconds());
            if (!cfg::kBagUseMsgStamp) { ps->header.stamp = stamp; }

            BagMessage bm;
            bm.stamp    = stamp;
            bm.role     = BagMsgRole::Pose;
            bm.pose_msg = poseStampedToPoseWithCov(ps,
                cfg::kBagPoseVarM, cfg::kBagPoseYawVarRad);
            all.push_back(bm);
        }
        // ── Odometry ─────────────────────────────────────────────────────────
        else if (serialised->topic_name == odom_topic) {
            rclcpp::SerializedMessage ser_msg(*serialised->serialized_data);
            auto odom = std::make_shared<nav_msgs::msg::Odometry>();
            odom_deser.deserialize_message(&ser_msg, odom.get());

            odom_deltas.push_back(rclcpp::Time(odom->header.stamp).seconds() - stamp.seconds());
            if (!cfg::kBagUseMsgStamp) { odom->header.stamp = stamp; }

            BagMessage bm;
            bm.stamp    = stamp;
            bm.role     = BagMsgRole::Odom;
            bm.odom_msg = *odom;
            all.push_back(bm);
        }
        // ── /tf ──────────────────────────────────────────────────────────────
        else if (serialised->topic_name == tf_topic) {
            rclcpp::SerializedMessage ser_msg(*serialised->serialized_data);
            auto tfm = std::make_shared<tf2_msgs::msg::TFMessage>();
            tf_deser.deserialize_message(&ser_msg, tfm.get());

            // Accumulate the latest map→odom and odom→gt_frame transforms.
            for (const auto & tx : tfm->transforms) {
                if (tx.header.frame_id == "map" && tx.child_frame_id == "odom") {
                    latest_map_odom = tx; have_map_odom = true;
                }
                if (tx.header.frame_id == "odom" && tx.child_frame_id == gt_frame) {
                    latest_odom_target = tx; have_odom_target = true;
                }
            }

            // Chain to get map→target as soon as both halves are available.
            if (have_map_odom && have_odom_target) {
                double x, y, yaw;
                composeMapToTarget(latest_map_odom, latest_odom_target, x, y, yaw);
                gt_index.push_back({stamp, x, y, yaw});
            }
        }
    }

    reader.close();

    // Sort by stamp (reader promises sequential but multi-topic can interleave).
    std::sort(all.begin(), all.end(),
        [](const BagMessage & a, const BagMessage & b) { return a.stamp < b.stamp; });
    std::sort(gt_index.begin(), gt_index.end(),
        [](const GtSample & a, const GtSample & b) { return a.stamp < b.stamp; });

    auto log_stats = [](const char* label, const std::vector<double>& deltas) {
        if (deltas.empty()) return;
        double mean = std::accumulate(deltas.begin(), deltas.end(), 0.0) / deltas.size();
        double var  = 0.0;
        for (double d : deltas) var += (d - mean) * (d - mean);
        double stdv = std::sqrt(var / deltas.size());
        auto [mn, mx] = std::minmax_element(deltas.begin(), deltas.end());
        RCLCPP_INFO(rclcpp::get_logger("bag_helpers"),
            "[bag_helpers] %s stamp delta: mean=%.4f s  min=%.4f s  max=%.4f s  std=%.4f s  (n=%zu%s)",
            label, mean, *mn, *mx, stdv, deltas.size(),
            cfg::kBagUseMsgStamp ? ", using msg stamp" : ", using bag stamp");
    };
    log_stats("Pose", pose_deltas);
    log_stats("Odometry", odom_deltas);

    RCLCPP_INFO(rclcpp::get_logger("bag_helpers"),
        "[bag_helpers] read %zu messages (%zu gt samples) from %s",
        all.size(), gt_index.size(), bag_path.c_str());
    return all;
}

/// Find the GtSample closest to `stamp` (not later than it).
inline const GtSample * closestGt(const std::vector<GtSample> & gt, const rclcpp::Time & stamp)
{
    if (gt.empty()) return nullptr;
    auto it = std::lower_bound(gt.begin(), gt.end(), stamp,
        [](const GtSample & s, const rclcpp::Time & t) { return s.stamp < t; });
    if (it == gt.begin()) return &gt.front();
    if (it == gt.end())   return &gt.back();
    const auto prev = it - 1;
    return ((stamp - prev->stamp).seconds() < (it->stamp - stamp).seconds()) ? &(*prev) : &(*it);
}
