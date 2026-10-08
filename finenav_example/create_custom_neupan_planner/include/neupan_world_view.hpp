// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// The map this bridge is registered against.  It is never dereferenced: it only
// exists to satisfy FINENAV_REGISTER_WORLDVIEW's map-argument list, exactly as
// create_custom_world_view does with the same type.  See the .cpp for why the
// registration needs at least one map type and why obstacle points cannot come
// from DummyMap.
#include "dummy_map.hpp"

#include "neupan_planner.hpp"

#include "finenav_example/neupan_world_view_params.hpp"

namespace finenav::example {

/**
 * @brief NeuPanWorldView — the bridge that turns a sensor stream into the
 *        obstacle points NeuPAN consumes.
 *
 * Implements NeuPanPlanner::IWorldView and is registered as "neupan_world_view",
 * so the layer is assembled with
 * `engine->createControlLayerByWorldView("neupan_world_view", "my_layer", policy)`
 * and the end user never names a planner or a map.  The framework builds this
 * object once per layer — inside createControlLayerByWorldView — and keeps it
 * alive as long as the layer, which is why it may own a subscription.
 *
 * === Where the processing happens, and why ===
 *
 * The layer's private sub-node is spun on its own thread by
 * finenav_util::NodeThread, but that SAME node and thread also carry the
 * framework's TaskChannel (command + feedback) traffic.  A slow subscription
 * callback here would therefore stall the layer's inputs.  So the split is:
 *
 *   - the subscription callback only filters (angle/range/downsample) and copies
 *     the beams into a raw buffer under a mutex — no TF, no allocation-heavy work;
 *   - prepare() — called once per plan(), on the layer's own worker thread,
 *     inside the map read lock and before plan() — does the TF transform into
 *     the world-fixed frame and the optional nearest-N truncation.
 *
 * === Staleness ===
 *
 * A frozen sensor must not read as "the path is clear".  If no message arrived
 * within points_timeout, prepare() publishes an empty point set and NeuPAN falls
 * back to pure path tracking, which is the same safe degradation the upstream
 * node implements with `scan_timeout`.
 */
class NeuPanWorldView : public NeuPanPlanner::IWorldView {
public:
    /// Parameter channel — the YAML namespace is the registration name.
    using ConfigType = neupan_world_view::Params;

    /**
     * @brief Constructor convention: the layer's private sub-node FIRST, then
     *        one `const Map&` per map type declared at registration.
     *
     * @param node the layer's private node, already spun; subscriptions and the
     *             TF listener are created on it here and in configure().
     * @param map  the DummyMap this view is registered against.  Unused on
     *             purpose — see the .cpp.
     */
    NeuPanWorldView(rclcpp::Node::SharedPtr node, const example::DummyMap& map);

    // ── MANDATORY: parameter channel ──────────────────────────────────────
    /**
     * @brief Apply the parameters, called once at startup and on every
     *        `ros2 param set`.
     * @details The subscription is installed here rather than in the
     *          constructor, because the framework constructs the view first and
     *          only then runs this: the topic is not known in the constructor.
     *          It is re-created only when points_topic or points_type actually
     *          changed, so an unrelated `ros2 param set` is free.
     */
    void configure(const ConfigType& cfg);

    // ── MANDATORY: per-plan derivation hook ───────────────────────────────
    /// Called once per plan(), inside the map read lock, before plan().
    /// This is where the world-frame conversion and the truncation happen.
    void prepare(const finenav::core::PlanningContext<NeuPanPlanner>& ctx);

    // ── Query path: must be cheap ─────────────────────────────────────────
    /// World-fixed obstacle points left behind by prepare().  No locking: it is
    /// written by prepare() and read by plan() on the same worker thread.
    const std::vector<finenav::common::Position3D>& obstaclePoints() const override {
        return points_;
    }

    // ── OPTIONAL injection ────────────────────────────────────────────────
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

private:
    /// One snapshot of raw sensor-frame points, as handed over by the callback.
    struct RawSnapshot {
        std::string                            frame_id;
        std::vector<finenav::common::Position3D> points;
        /// False for a planar source (LaserScan, or a cloud with no z field):
        /// the world-frame height band is then meaningless and is skipped.
        bool                                   has_z{true};
        std::chrono::steady_clock::time_point  received{};
        bool                                   valid{false};
    };

    void installSubscription(const std::string& topic, const std::string& type);
    void eraseSubscription();

    void onLaserScan(const sensor_msgs::msg::LaserScan& msg);
    void onPointCloud2(const sensor_msgs::msg::PointCloud2& msg);

    /// Copy a filtered point list into the shared raw buffer.
    void storeRaw(std::string frame_id, std::vector<finenav::common::Position3D> pts,
                  bool has_z);

    /// Take the latest snapshot out of the shared buffer.
    RawSnapshot takeSnapshot();

    /// Drop points outside [min_height_, max_height_] (world-frame z, inclusive).
    void applyHeightBand();

    rclcpp::Node::SharedPtr node_;

    /// Exactly one of these is non-null, decided by points_type.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    /// Guards raw_points_ / raw_frame_ / raw_received_ / have_raw_, which are
    /// written by the node thread and consumed by the layer's worker thread.
    std::mutex mutex_;
    std::vector<finenav::common::Position3D> raw_points_;
    std::string                              raw_frame_;
    bool                                     raw_has_z_{true};
    std::chrono::steady_clock::time_point    raw_received_{};
    bool                                     have_raw_{false};

    /// Written by prepare() and read by plan(), both on the worker thread.
    std::vector<finenav::common::Position3D> points_;

    // Cached parameters.  Only the fields actually used are kept: the generated
    // Params struct carries an rclcpp::Time stamp, so keeping the whole thing
    // would be misleading.
    std::string points_frame_;
    double scan_angle_min_{-3.14159265358979};
    double scan_angle_max_{3.14159265358979};
    double scan_range_min_{0.0};
    double scan_range_max_{5.0};
    int    scan_downsample_{1};
    bool   flip_angle_{false};
    double points_timeout_{0.5};
    int    max_points_{0};

    /// World-frame height band, applied after the TF transform.  This is what
    /// removes the floor returns a 3D lidar produces everywhere beyond a few
    /// metres — without it NeuPAN sees a ring of phantom obstacles.  See
    /// neupan_world_view.yaml for the full explanation.
    double min_height_{0.15};
    double max_height_{2.0};

    /// What the live subscription was installed for, so configure() can tell
    /// whether it needs re-creating.
    std::string installed_topic_;
    std::string installed_type_;

    rclcpp::Logger logger_{rclcpp::get_logger("NeuPanWorldView")};

    /// System time, not ROS time: a warning must still be rate-limited before
    /// /clock exists.
    rclcpp::Clock clock_{RCL_SYSTEM_TIME};
};

}  // namespace finenav::example
