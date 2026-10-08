// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "neupan_world_view.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {

namespace {

/// Full SE(3) transform of a point set, in place.
///
/// NOT a 2D yaw+translation shortcut: the world-frame height band that turns a
/// 3D cloud into NeuPAN's 2D obstacle set is only meaningful if z comes out
/// right, and z depends on the transform's z translation.  With a plain SE(2)
/// transform every floor return would keep its sensor-frame z and the band
/// would classify the whole cloud wrongly.
void transformInPlace(std::vector<finenav::common::Position3D>& pts,
                      const geometry_msgs::msg::Transform& tf)
{
    // Matrix3x3(Quaternion) normalises the quaternion for us.
    tf2::Quaternion q(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
    const tf2::Matrix3x3 basis(q);
    const tf2::Vector3 translation(tf.translation.x, tf.translation.y, tf.translation.z);

    for (auto& p : pts) {
        const tf2::Vector3 v = basis * tf2::Vector3(p.x, p.y, p.z) + translation;
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
    }
}

/// The frame the points are actually expressed in.
///
/// points_frame wins when set.  Otherwise use the message's own header, and
/// fall back to the world-fixed frame when the sensor declares none — a scan
/// with no frame id is only interpretable as already being world-fixed.
std::string sourceFrame(const std::string& configured,
                        const std_msgs::msg::Header& header)
{
    if (!configured.empty()) return configured;
    if (!header.frame_id.empty()) return header.frame_id;
    return finenav::common::frames().world_fixed;
}

}  // namespace

NeuPanWorldView::NeuPanWorldView(rclcpp::Node::SharedPtr node,
                                 const example::DummyMap& /*map*/)
    : node_(std::move(node))
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

    // The listener is constructed against its own internal node and thread
    // rather than against node_ on purpose: node_ is already spun by the
    // layer's NodeThread, and handing the same node to a second executor
    // (which the node-taking constructor may do when spin_thread is left on)
    // would make rclcpp refuse to add it twice.  This also keeps a blocking TF
    // subscription off the thread that carries the layer's TaskChannel traffic.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // The map is deliberately not read: FineNav's only registered map in this
    // repository is the inert example::DummyMap, which carries no geometry.
    // Obstacles come from the subscription instead.  Replace this with a real
    // map dependency (take it as a constructor argument and declare it in
    // FINENAV_REGISTER_WORLDVIEW) once such a map plugin exists.
}

// ==============================================================================
//  Parameter channel
// ==============================================================================
void NeuPanWorldView::configure(const ConfigType& cfg) {
    points_frame_    = cfg.points_frame;
    scan_angle_min_  = cfg.scan_angle_min;
    scan_angle_max_  = cfg.scan_angle_max;
    scan_range_min_  = cfg.scan_range_min;
    scan_range_max_  = cfg.scan_range_max;
    scan_downsample_ = std::max(1, static_cast<int>(cfg.scan_downsample));
    flip_angle_      = cfg.flip_angle;
    points_timeout_  = cfg.points_timeout;
    max_points_      = static_cast<int>(cfg.max_points);
    min_height_      = cfg.min_height;
    max_height_      = cfg.max_height;

    if (min_height_ > max_height_) {
        RCLCPP_WARN(logger_,
            "min_height (%.2f) is above max_height (%.2f): every point will be "
            "discarded and NeuPAN will receive no obstacles. Swapping them.",
            min_height_, max_height_);
        std::swap(min_height_, max_height_);
    }

    // The subscription cannot be created in the constructor: the framework
    // builds the view first and only then calls configure(), so this is the
    // first moment the topic is known.  Re-create it only when the identity
    // actually changed, so an unrelated `ros2 param set` costs nothing.
    if (cfg.points_topic != installed_topic_ || cfg.points_type != installed_type_) {
        installSubscription(cfg.points_topic, cfg.points_type);
    }
}

void NeuPanWorldView::eraseSubscription() {
    scan_sub_.reset();
    cloud_sub_.reset();
    installed_topic_.clear();
    installed_type_.clear();
}

void NeuPanWorldView::installSubscription(const std::string& topic,
                                          const std::string& type) {
    eraseSubscription();

    if (topic.empty()) {
        RCLCPP_WARN(logger_,
            "points_topic is empty; NeuPAN will receive no obstacle points and will "
            "only track the reference path.");
        return;
    }

    if (type == "laser_scan") {
        scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan& msg) { onLaserScan(msg); });
    } else if (type == "point_cloud2") {
        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2& msg) { onPointCloud2(msg); });
    } else {
        RCLCPP_ERROR(logger_,
            "points_type '%s' is not supported (use 'laser_scan' or 'point_cloud2'); "
            "NeuPAN will receive no obstacle points.", type.c_str());
        return;
    }

    installed_topic_ = topic;
    installed_type_  = type;
    RCLCPP_INFO(logger_, "Obstacle source: '%s' (%s).", topic.c_str(), type.c_str());
}

// ==============================================================================
//  Subscription callbacks — cheap by design
//
//  These run on the layer's private-node thread, which also carries the
//  framework's TaskChannel traffic, so they must not block: filter, copy, done.
//  The TF lookup lives in prepare(), on the layer's worker thread.
// ==============================================================================
void NeuPanWorldView::storeRaw(std::string frame_id,
                               std::vector<finenav::common::Position3D> pts,
                               bool has_z) {
    std::lock_guard<std::mutex> lock(mutex_);
    raw_frame_    = std::move(frame_id);
    raw_points_   = std::move(pts);
    raw_has_z_    = has_z;
    raw_received_ = std::chrono::steady_clock::now();
    have_raw_     = true;
}

NeuPanWorldView::RawSnapshot NeuPanWorldView::takeSnapshot() {
    RawSnapshot snap;
    std::lock_guard<std::mutex> lock(mutex_);
    // Copy rather than move: the callback keeps its capacity, so the next
    // message does not have to grow the buffer again from scratch.
    snap.frame_id = raw_frame_;
    snap.points   = raw_points_;
    snap.has_z    = raw_has_z_;
    snap.received = raw_received_;
    snap.valid    = have_raw_;
    return snap;
}

void NeuPanWorldView::onLaserScan(const sensor_msgs::msg::LaserScan& msg) {
    const int n = static_cast<int>(msg.ranges.size());
    if (n <= 0) return;

    // Prefer the message's own increment; deriving it from
    // (angle_max - angle_min) / n divides by one beam too few.
    const double angle_inc =
        msg.angle_increment != 0.0f
            ? static_cast<double>(msg.angle_increment)
            : (n > 1 ? (static_cast<double>(msg.angle_max) - msg.angle_min) / (n - 1) : 0.0);

    std::vector<finenav::common::Position3D> pts;
    pts.reserve(static_cast<std::size_t>(n / scan_downsample_) + 1);

    for (int i = 0; i < n; i += scan_downsample_) {
        const double r = msg.ranges[static_cast<std::size_t>(i)];
        if (!std::isfinite(r)) continue;
        if (r < scan_range_min_ || r > scan_range_max_) continue;

        const double angle = flip_angle_
            ? static_cast<double>(msg.angle_max) - i * angle_inc
            : static_cast<double>(msg.angle_min) + i * angle_inc;
        if (angle <= scan_angle_min_ || angle >= scan_angle_max_) continue;

        pts.push_back({r * std::cos(angle), r * std::sin(angle), 0.0});
    }

    // A LaserScan is planar: its z is the sensor plane, not an obstacle height,
    // so the height band must not be applied to it.
    storeRaw(sourceFrame(points_frame_, msg.header), std::move(pts), /*has_z=*/false);
}

void NeuPanWorldView::onPointCloud2(const sensor_msgs::msg::PointCloud2& msg) {
    std::vector<finenav::common::Position3D> pts;
    bool has_z = false;
    try {
        sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");

        // A cloud without z is planar; carry x/y and let prepare() skip the
        // height band rather than filtering every point away.
        has_z = std::any_of(msg.fields.begin(), msg.fields.end(),
            [](const sensor_msgs::msg::PointField& f) { return f.name == "z"; });

        const std::size_t count =
            static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
        pts.reserve(count);

        if (has_z) {
            sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
            for (std::size_t i = 0; i < count; ++i, ++ix, ++iy, ++iz) {
                const double x = *ix;
                const double y = *iy;
                const double z = *iz;
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
                if (std::hypot(x, y) < scan_range_min_ ||
                    std::hypot(x, y) > scan_range_max_) continue;
                pts.push_back({x, y, z});
            }
        } else {
            RCLCPP_WARN_THROTTLE(logger_, clock_, 5000,
                "Cloud on '%s' has no 'z' field; carrying x/y only and skipping the "
                "height band. Point a 3D lidar source here, or use points_type "
                "'laser_scan' for a planar sensor.", installed_topic_.c_str());
            for (std::size_t i = 0; i < count; ++i, ++ix, ++iy) {
                const double x = *ix;
                const double y = *iy;
                if (!std::isfinite(x) || !std::isfinite(y)) continue;
                if (std::hypot(x, y) < scan_range_min_ ||
                    std::hypot(x, y) > scan_range_max_) continue;
                pts.push_back({x, y, 0.0});
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(logger_, clock_, 2000,
            "Cloud on '%s' is not a usable float XYZ point cloud: %s",
            installed_topic_.c_str(), e.what());
        return;
    }

    storeRaw(sourceFrame(points_frame_, msg.header), std::move(pts), has_z);
}

// ==============================================================================
//  Per-plan derivation — the only place heavy work belongs
// ==============================================================================

void NeuPanWorldView::applyHeightBand() {
    const auto outside = [this](const finenav::common::Position3D& p) {
        return p.z < min_height_ || p.z > max_height_;
    };
    points_.erase(std::remove_if(points_.begin(), points_.end(), outside),
                  points_.end());
}

void NeuPanWorldView::prepare(
    const finenav::core::PlanningContext<NeuPanPlanner>& ctx) {
    points_.clear();

    const RawSnapshot snap = takeSnapshot();
    if (!snap.valid || snap.points.empty()) {
        // Nothing received yet, or the last message carried no usable beam.
        // An empty set is the honest answer, not a stale one.
        return;
    }

    const double age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - snap.received).count();
    if (age > points_timeout_) {
        RCLCPP_WARN_THROTTLE(logger_, clock_, 1000,
            "No obstacle points for %.2f s (points_timeout %.2f s); treating the "
            "environment as unknown and leaving NeuPAN to track the reference only.",
            age, points_timeout_);
        return;
    }

    points_ = snap.points;

    const std::string& world = finenav::common::frames().world_fixed;
    if (snap.frame_id != world) {
        try {
            // The LATEST transform, not the message stamp: the buffer's history
            // is short, and a sensor that lags TF would otherwise throw here on
            // every single cycle.
            const auto tf = tf_buffer_->lookupTransform(
                world, snap.frame_id, tf2::TimePointZero);
            transformInPlace(points_, tf.transform);
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(logger_, clock_, 2000,
                "Cannot transform obstacle points from '%s' to '%s' (%s); dropping "
                "them for this cycle.", snap.frame_id.c_str(), world.c_str(), e.what());
            points_.clear();
            return;
        }
    }

    // Collapse 3D -> 2D: NeuPAN consumes an (x, y) set, so a floor that the
    // lidar sees everywhere beyond a few metres would otherwise read as a ring
    // of obstacles.  Done on world-frame z, so it does not care how the sensor
    // is mounted.  Skipped for a planar source, whose z is the sensor plane.
    if (snap.has_z) applyHeightBand();

    // Optional performance guard: keep the points nearest the robot.  Leave it
    // at 0, or at or above the planner's dune_max_num, so that it never changes
    // what the algorithm sees — only how much of it is handed over.
    if (max_points_ > 0 && static_cast<int>(points_.size()) > max_points_) {
        const double rx = ctx.robot_state.pose.position.x;
        const double ry = ctx.robot_state.pose.position.y;
        const auto keep = static_cast<std::size_t>(max_points_);

        std::nth_element(
            points_.begin(),
            points_.begin() + static_cast<std::ptrdiff_t>(keep),
            points_.end(),
            [rx, ry](const finenav::common::Position3D& a,
                     const finenav::common::Position3D& b) {
                return std::hypot(a.x - rx, a.y - ry) < std::hypot(b.x - rx, b.y - ry);
            });

        points_.resize(keep);
    }
}

}  // namespace finenav::example

// One-line registration.  The macro records only TYPES (planner + maps); the
// framework resolves them back to their registered algorithm names
// ("neupan_planner" / "dummy_map") at assembly time, so this file never writes
// those names by hand and plugin load order is irrelevant.
//
// The map argument is example::DummyMap for want of any other Map in this
// repository: the bridge is registered against it so that
// FINENAV_REGISTER_WORLDVIEW has a map type to declare, and it never reads it.
// Swap in a real map plugin here once one exists.
FINENAV_REGISTER_WORLDVIEW(
    finenav::example::NeuPanWorldView,
    neupan_world_view,
    finenav::example::NeuPanPlanner,
    finenav::example::DummyMap)
