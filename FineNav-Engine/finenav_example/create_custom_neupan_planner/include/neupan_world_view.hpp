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
 * `engine->createControlLayer("neupan_world_view", "my_layer", policy)`
 * and the end user never names a planner or a map.  The framework builds this
 * object once per layer — inside createControlLayer — and keeps it
 * alive as long as the layer, which is why it may own a subscription.
 *
 * === Where the processing happens, and why ===
 *
 * The layer's private sub-node is spun on its own thread by
 * finenav_util::NodeThread, but that SAME node and thread also carry the
 * framework's TaskChannel (command + feedback) traffic.  A slow subscription
 * callback here would therefore stall the layer's inputs.  So the split is:
 *
 *   - the subscription callback caches an immutable message under a mutex;
 *   - prepare() — called once per plan(), on the layer's own worker thread,
 *     inside the map read lock and before plan() — does the TF transform into
 *     the world-fixed frame and the optional nearest-N truncation.
 *
 * === Staleness ===
 *
 * A frozen sensor must not read as "the path is clear".  If no message arrived
 * within points_timeout, prepare() marks the observation invalid. The planner
 * stops immediately and reports a persistent outage to FineNav.
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

    bool observationValid() const override { return valid_; }
    const std::vector<finenav::common::Position3D>& collisionPoints() const override {
        return collision_points_;
    }

    // ── OPTIONAL injection ────────────────────────────────────────────────
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

private:
    friend struct NeuPanWorldViewTestAccess;
    // Callbacks only publish immutable messages under this lock. All decoding,
    // configuration reads and TF work belong to the planning worker thread.
    void installSubscription(const std::string& topic, const std::string& type);
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::mutex mutex_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
    std::chrono::steady_clock::time_point received_{};
    std::uint64_t generation_{0};
    ConfigType cfg_;
    bool valid_{false};
    std::vector<finenav::common::Position3D> points_, collision_points_;
    rclcpp::Logger logger_{rclcpp::get_logger("NeuPanWorldView")};
    rclcpp::Clock clock_{RCL_SYSTEM_TIME};
};

} // namespace finenav::example
