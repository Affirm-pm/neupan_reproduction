// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/control_layer.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_navigator/navigator.hpp"
#include "finenav_util/common/log_macros.hpp"

namespace finenav {

/**
 * @brief FineNavEngine — factory + connector for the navigation framework.
 *
 * ## Usage flow
 * 1. createMap<MapT>(name)          — user keeps the returned shared_ptr
 * 2. createLayer<PlannersT>(name)   — user keeps the returned shared_ptr
 * 3. layer->bind_map_view(map, ...) — user connects map to layer
 * 4. connect(in, layer, out)        — engine wires the pipeline topics
 *
 * The engine does NOT store user resources.  The user owns the shared_ptrs
 * and is responsible for their lifetime.  Templates parameters stay at the
 * call site and never propagate through the engine's own type.
 */
class FineNavEngine {
public:
    // ── Lifecycle ──────────────────────────────────────────────────────────

    explicit FineNavEngine(const std::shared_ptr<rclcpp::Node>& node);

    FineNavEngine(const FineNavEngine&) = delete;
    FineNavEngine& operator=(const FineNavEngine&) = delete;
    FineNavEngine(FineNavEngine&&) = default;
    FineNavEngine& operator=(FineNavEngine&&) = default;
    ~FineNavEngine() = default;

    // ── Accessors ──────────────────────────────────────────────────────────

    const std::shared_ptr<rclcpp::Node>& getNode() const { return node_; }
    FineNavLocalizer* getLocalizer() { return localizer_.get(); }
    Navigator* getNavigator() { return navigator_.get(); }

    // ── Factory: map ──────────────────────────────────────────────────────

    /**
     * @brief Create a named MapServer.  The caller owns the returned pointer.
     * @tparam MapT  Must satisfy SpatialContainer.
     */
    template <typename MapT>
    [[nodiscard("MapServer created but handle was lost!")]]
    std::shared_ptr<finenav::core::MapServer<MapT>> createMapServer(const std::string& name);

    // ── Factory: layer ────────────────────────────────────────────────────

    /**
     * @brief Create a named ControlLayer.  The caller owns the returned pointer.
     * @tparam PlannersT       PlannerSet<...> listing supported planners.
     * @tparam ExecutionPolicy  SingleShotPolicy or ContinuousPolicy.
     */
    template <typename PlannersT, typename ExecutionPolicy = finenav::core::SingleShotPolicy>
    [[nodiscard("ControlLayer created but handle was lost!")]]
    std::shared_ptr<finenav::core::ControlLayer<PlannersT, ExecutionPolicy>> createControlLayer(
        const std::string& name, ExecutionPolicy policy = ExecutionPolicy{});

    // ── Connector ─────────────────────────────────────────────────────────

    struct Pipeline {
        std::string input;
        std::string output;
    };

    /**
     * @brief Wire one or more ControlLayers into a sequential pipeline.
     *        The first layer subscribes to @p p.input; the last publishes
     *        on @p p.output.  Intermediate topics are auto-generated.
     */
    template <typename... LayersT>
    void connectPipeline(const Pipeline& p, std::shared_ptr<LayersT>... layers);

private:
    template <typename FirstT, typename SecondT, typename... RestT>
    void wire_layers(std::shared_ptr<FirstT> first,
                     std::shared_ptr<SecondT> second,
                     std::shared_ptr<RestT>... rest);

    std::shared_ptr<rclcpp::Node> node_;

    // ── Engine-level singletons ────────────────────────────────────────────
    std::unique_ptr<FineNavLocalizer> localizer_;
    std::unique_ptr<Navigator>        navigator_;
};

// ══════════════════════════════════════════════════════════════════════════════
//  Constructor
// ══════════════════════════════════════════════════════════════════════════════

inline FineNavEngine::FineNavEngine(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node) {
    FINENAV_LOG_INFO(node_->get_logger(), "Initializing...");

    localizer_ = std::make_unique<FineNavLocalizer>(node_);
    FINENAV_LOG_INFO(node_->get_logger(), "Localizer ready.");

    navigator_ = std::make_unique<Navigator>(node_);

    navigator_->set_robot_pose_provider([this]() -> geometry_msgs::msg::Pose {
        const auto& state = localizer_->getState();
        geometry_msgs::msg::Pose p;
        p.position.x    = state.p.x();
        p.position.y    = state.p.y();
        p.position.z    = state.p.z();
        const auto q    = state.R.unit_quaternion();
        p.orientation.w = q.w();
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        return p;
    });

    navigator_->set_robot_twist_provider([this]() -> geometry_msgs::msg::Twist {
        const auto& state = localizer_->getState();
        geometry_msgs::msg::Twist t;
        t.linear.x  = state.v.x();
        t.linear.y  = state.v.y();
        t.linear.z  = state.v.z();
        t.angular.x = state.omega.x();
        t.angular.y = state.omega.y();
        t.angular.z = state.omega.z();
        return t;
    });
    FINENAV_LOG_INFO(node_->get_logger(), "Navigator ready.");

    FINENAV_LOG_INFO(node_->get_logger(), "All singletons initialized.");
}

// ══════════════════════════════════════════════════════════════════════════════
//  Factory: map
// ══════════════════════════════════════════════════════════════════════════════

template <typename MapT>
std::shared_ptr<finenav::core::MapServer<MapT>> FineNavEngine::createMapServer(
    const std::string& name) {
    return std::make_shared<finenav::core::MapServer<MapT>>(
        node_, name, localizer_.get());
}

// ══════════════════════════════════════════════════════════════════════════════
//  Factory: layer
// ══════════════════════════════════════════════════════════════════════════════

template <typename PlannersT, typename ExecutionPolicy>
std::shared_ptr<finenav::core::ControlLayer<PlannersT, ExecutionPolicy>>
FineNavEngine::createControlLayer(const std::string& name, ExecutionPolicy policy) {
    auto layer = std::make_shared<finenav::core::ControlLayer<PlannersT, ExecutionPolicy>>(
        node_, name, std::move(policy));
    layer->set_robot_state_provider([this]() -> finenav::core::RobotState {
        return localizer_->getRobotState().value_or(finenav::core::RobotState{});
    });
    return layer;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Connector
// ══════════════════════════════════════════════════════════════════════════════

template <typename... LayersT>
void FineNavEngine::connectPipeline(const Pipeline& p,
                            std::shared_ptr<LayersT>... layers) {
    static_assert(sizeof...(layers) >= 1, "connectPipeline() requires at least one layer!");
    auto layer_tuple = std::make_tuple(layers...);

    std::get<0>(layer_tuple)->setInputTopic(p.input);

    constexpr auto last_idx = sizeof...(layers) - 1;
    std::get<last_idx>(layer_tuple)->setOutputTopic(p.output);

    if constexpr (sizeof...(layers) >= 2) {
        wire_layers(layers...);
    }

    (layers->setupDataPorts(), ...);
}

template <typename FirstT, typename SecondT, typename... RestT>
void FineNavEngine::wire_layers(std::shared_ptr<FirstT> first,
                                std::shared_ptr<SecondT> second,
                                std::shared_ptr<RestT>... rest) {
    std::string internal_topic = first->getName() + "_to_" + second->getName();
    FINENAV_LOG_INFO(node_->get_logger(),
        "Connecting {} -> {} via topic: {}",
        first->getName(), second->getName(), internal_topic);
    first->setOutputTopic(internal_topic);
    second->setInputTopic(internal_topic);
    wire_layers(second, rest...);
}

}  // namespace finenav
