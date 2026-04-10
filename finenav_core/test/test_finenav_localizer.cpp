// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <Eigen/Geometry>

#include "finenav_core/localization/finenav_localizer.hpp"

using namespace std::chrono_literals;

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Build a diagonal 6×6 covariance array (ROS flat 36-element row-major).
static std::array<double, 36> diag6(double cx, double cy, double cz,
                                    double crx, double cry, double crz)
{
    std::array<double, 36> cov{};
    cov.fill(0.0);
    cov[0]  = cx;
    cov[7]  = cy;
    cov[14] = cz;
    cov[21] = crx;
    cov[28] = cry;
    cov[35] = crz;
    return cov;
}

/// Build a unit quaternion from yaw angle (rotation around Z axis).
static geometry_msgs::msg::Quaternion quatFromYaw(double yaw_rad)
{
    geometry_msgs::msg::Quaternion q;
    q.w = std::cos(yaw_rad / 2.0);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw_rad / 2.0);
    return q;
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_finenav_localizer");

    RCLCPP_INFO(node->get_logger(), "════════════════════════════════════════════");
    RCLCPP_INFO(node->get_logger(), "  FineNavLocalizer — smoke test");
    RCLCPP_INFO(node->get_logger(), "════════════════════════════════════════════");

    // ── 1. Construct localizer ────────────────────────────────────────────────
    finenav::LocalizerOptions opts;
    opts.update_rate_hz        = 50.0;   // 50 Hz filter loop
    opts.max_delay_sec         = 0.3;    // accept observations up to 300 ms old
    opts.pose_smoothing_steps  = 3;      // pose R scaled ×3 → smoother tracking
    opts.twist_smoothing_steps = 1;      // twist at full weight
    opts.pose_gate_dist        = 12.59;  // chi^2(6) 95 %
    opts.twist_gate_dist       = 12.59;
    opts.model_opts.vel_noise_std   = 0.05;
    opts.model_opts.omega_noise_std = 0.05;

    finenav::FineNavLocalizer localizer(node, opts);
    RCLCPP_INFO(node->get_logger(), "[1] Localizer constructed.");

    // ── 2. Set initial pose ───────────────────────────────────────────────────
    //  Position: p = [1.0, 2.0, 0.0]
    //  Heading:  yaw = 45 deg
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = node->now();
        msg.header.frame_id = "map";

        msg.pose.pose.position.x  = 1.0;
        msg.pose.pose.position.y  = 2.0;
        msg.pose.pose.position.z  = 0.0;
        msg.pose.pose.orientation = quatFromYaw(M_PI / 4.0);

        const auto cov = diag6(0.10, 0.10, 0.01, 0.01, 0.01, 0.05);
        std::copy(cov.begin(), cov.end(), msg.pose.covariance.begin());

        localizer.setInitialPose(msg);
        RCLCPP_INFO(node->get_logger(),
            "[2] setInitialPose  p=[1.0, 2.0, 0.0]  yaw=45°");
    }

    // Let the filter spin for a bit before feeding observations
    rclcpp::sleep_for(120ms);

    // ── 3. Feed a current pose observation (e.g. from SLAM / visual odometry) ─
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = node->now();   // no delay
        msg.header.frame_id = "map";

        msg.pose.pose.position.x  = 1.30;
        msg.pose.pose.position.y  = 2.08;
        msg.pose.pose.position.z  = 0.00;
        msg.pose.pose.orientation = quatFromYaw(M_PI / 4.0 + 0.05);  // slight drift

        const auto cov = diag6(0.05, 0.05, 0.01, 0.005, 0.005, 0.02);
        std::copy(cov.begin(), cov.end(), msg.pose.covariance.begin());

        localizer.feedPose(msg);
        RCLCPP_INFO(node->get_logger(),
            "[3] feedPose (current)  p=[1.30, 2.08, 0.00]  delay≈0 ms");
    }

    // ── 4. Feed a twist observation (e.g. from wheel odometry) ───────────────
    //  Velocities are in the body frame (base_link).
    {
        geometry_msgs::msg::TwistWithCovarianceStamped msg;
        msg.header.stamp    = node->now();
        msg.header.frame_id = "base_link";

        msg.twist.twist.linear.x  =  0.50;   // 0.5 m/s forward
        msg.twist.twist.linear.y  =  0.00;
        msg.twist.twist.linear.z  =  0.00;
        msg.twist.twist.angular.x =  0.00;
        msg.twist.twist.angular.y =  0.00;
        msg.twist.twist.angular.z =  0.05;   // gentle left turn

        const auto cov = diag6(0.01, 0.01, 0.001, 0.001, 0.001, 0.005);
        std::copy(cov.begin(), cov.end(), msg.twist.covariance.begin());

        localizer.feedTwist(msg);
        RCLCPP_INFO(node->get_logger(),
            "[4] feedTwist (current)  vx=0.50 m/s  wz=0.05 rad/s");
    }

    // ── 5. Feed a delayed pose observation (e.g. 150 ms camera latency) ──────
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        // Back-date the stamp by 150 ms to simulate processing latency
        msg.header.stamp    = node->now() - rclcpp::Duration::from_seconds(0.15);
        msg.header.frame_id = "map";

        msg.pose.pose.position.x  = 1.18;
        msg.pose.pose.position.y  = 2.03;
        msg.pose.pose.position.z  = 0.00;
        msg.pose.pose.orientation = quatFromYaw(M_PI / 4.0);

        const auto cov = diag6(0.08, 0.08, 0.01, 0.005, 0.005, 0.03);
        std::copy(cov.begin(), cov.end(), msg.pose.covariance.begin());

        localizer.feedPose(msg);
        RCLCPP_INFO(node->get_logger(),
            "[5] feedPose (delayed)  p=[1.18, 2.03, 0.00]  delay=150 ms  "
            "→ sequential retrocorrection will be applied");
    }

    // ── 6. Feed a pose that should be REJECTED by the Mahalanobis gate ───────
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp    = node->now();
        msg.header.frame_id = "map";

        // Wildly inconsistent with the current estimate → d² >> threshold
        msg.pose.pose.position.x  = 100.0;
        msg.pose.pose.position.y  = 200.0;
        msg.pose.pose.position.z  =   0.0;
        msg.pose.pose.orientation = quatFromYaw(0.0);

        const auto cov = diag6(0.001, 0.001, 0.001, 0.001, 0.001, 0.001);
        std::copy(cov.begin(), cov.end(), msg.pose.covariance.begin());

        localizer.feedPose(msg);
        RCLCPP_INFO(node->get_logger(),
            "[6] feedPose (outlier)  p=[100, 200, 0]  "
            "→ expected WARN: rejected by Mahalanobis gate");
    }

    // Let the filter process all observations
    rclcpp::sleep_for(300ms);

    // ── 7. Print final state ──────────────────────────────────────────────────
    const finenav::NavStateD state   = localizer.getState();
    const auto               P       = localizer.getCovariance();
    const rclcpp::Time       t_last  = localizer.getLastUpdateTime();

    const Eigen::Quaterniond q_out(state.R.matrix());
    const double yaw_out = std::atan2(
        2.0 * (q_out.w() * q_out.z() + q_out.x() * q_out.y()),
        1.0 - 2.0 * (q_out.y() * q_out.y() + q_out.z() * q_out.z()));

    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "════ Final Filter State ════════════════════");
    RCLCPP_INFO(node->get_logger(),
        "  p       = [%.4f,  %.4f,  %.4f]  (m)",
        state.p.x(), state.p.y(), state.p.z());
    RCLCPP_INFO(node->get_logger(),
        "  yaw     = %.4f rad  (%.2f deg)",
        yaw_out, yaw_out * 180.0 / M_PI);
    RCLCPP_INFO(node->get_logger(),
        "  v_world = [%.4f,  %.4f,  %.4f]  (m/s)",
        state.v.x(), state.v.y(), state.v.z());
    RCLCPP_INFO(node->get_logger(),
        "  ω_body  = [%.4f,  %.4f,  %.4f]  (rad/s)",
        state.omega.x(), state.omega.y(), state.omega.z());
    RCLCPP_INFO(node->get_logger(),
        "  P diag  = p:[%.4f, %.4f, %.4f]  R:[%.4f, %.4f, %.4f]"
        "  v:[%.4f, %.4f, %.4f]  ω:[%.4f, %.4f, %.4f]",
        P(0,0),  P(1,1),  P(2,2),
        P(3,3),  P(4,4),  P(5,5),
        P(6,6),  P(7,7),  P(8,8),
        P(9,9),  P(10,10), P(11,11));
    RCLCPP_INFO(node->get_logger(),
        "  last update = %.4f s", t_last.seconds());
    RCLCPP_INFO(node->get_logger(), "════════════════════════════════════════════");

    rclcpp::shutdown();
    return 0;
}

