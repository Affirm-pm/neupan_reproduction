// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <algorithm>
#include <any>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "finenav_core/map/map_server.hpp"
#include "finenav_core/plan/control_layer.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_navigator/navigator.hpp"

namespace finenav {

class FineNavEngine {
public:
    // ── 生命周期 ─────────────────────────────────────────────────────────
    explicit FineNavEngine(const std::shared_ptr<rclcpp::Node>& node);

    FineNavEngine(const FineNavEngine&) = delete;
    FineNavEngine& operator=(const FineNavEngine&) = delete;
    FineNavEngine(FineNavEngine&&) = default;
    FineNavEngine& operator=(FineNavEngine&&) = default;
    ~FineNavEngine() = default;

    // ── 访问器 ───────────────────────────────────────────────────────────
    const std::shared_ptr<rclcpp::Node>& getNode() const { return node_; }
    FineNavLocalizer* getLocalizer() { return localizer_.get(); }
    Navigator* getNavigator() { return navigator_.get(); }

    // ── 模板成员（声明）──────────────────────────────────────────────────

    template <typename MapT>
    [[nodiscard("Map resource was created but the handle was lost!")]]
    std::shared_ptr<finenav::core::MapServer<MapT>> createMapResource(const std::string& name);

    template <typename MapT>
    [[nodiscard("Map resource was created but the handle was lost!")]]
    std::shared_ptr<finenav::core::MapServer<MapT>> getMapResource(const std::string& name);

    template <typename PlannersT, typename ExecutionPolicy = finenav::core::SingleShotPolicy>
    [[nodiscard("ControlLayer was created but the handle was lost!")]]
    std::shared_ptr<finenav::core::ControlLayer<PlannersT, ExecutionPolicy>> createControlLayer(
        const std::string& name, ExecutionPolicy policy = ExecutionPolicy{});

    struct PipelineIO {
        std::string input_topic;
        std::string output_topic;
    };

    template <typename... LayersT>
    void registerPipeline(const PipelineIO& io, std::shared_ptr<LayersT>... layers);

private:
    template <typename T>
    void wire_layers(std::shared_ptr<T>) {}

    template <typename FirstT, typename SecondT, typename... RestT>
    void wire_layers(std::shared_ptr<FirstT> first,
                     std::shared_ptr<SecondT> second,
                     std::shared_ptr<RestT>... rest);

    std::shared_ptr<rclcpp::Node> node_;

    // ── Engine-level singletons ──────────────────────────────────────────
    std::unique_ptr<FineNavLocalizer> localizer_;
    std::unique_ptr<Navigator>        navigator_;

    // ── Held resources ───────────────────────────────────────────────────
    std::vector<std::pair<std::string, std::any>> map_resources_;
    std::vector<std::shared_ptr<void>>            held_layers_;
};

// ══════════════════════════════════════════════════════════════════════════
//  构造函数
// ══════════════════════════════════════════════════════════════════════════

inline FineNavEngine::FineNavEngine(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "Initializing...");

    localizer_ = std::make_unique<FineNavLocalizer>(node_);
    RCLCPP_INFO(node_->get_logger(), "Localizer ready.");

    navigator_ = std::make_unique<Navigator>(node_);

    // 注入机器人位姿提供函数，供 BT 条件节点（IsGoalReached、IsStuck 等）读取
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

    // 注入机器人速度提供函数，供 IsGoalReached 等条件节点读取
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
    RCLCPP_INFO(node_->get_logger(), "Navigator ready.");

    RCLCPP_INFO(node_->get_logger(), "All singletons initialized.");
}

// ══════════════════════════════════════════════════════════════════════════
//  模板成员函数实现
// ══════════════════════════════════════════════════════════════════════════

template <typename MapT>
std::shared_ptr<finenav::core::MapServer<MapT>> FineNavEngine::createMapResource(
    const std::string& name) {
    auto it = std::find_if(map_resources_.begin(), map_resources_.end(),
                           [&](const auto& pair) { return pair.first == name; });
    if (it == map_resources_.end()) {
        auto new_map = std::make_shared<finenav::core::MapServer<MapT>>(
            node_, name, localizer_.get());
        map_resources_.emplace_back(name, new_map);
        return new_map;
    }
    return std::any_cast<std::shared_ptr<finenav::core::MapServer<MapT>>>(it->second);
}

template <typename MapT>
std::shared_ptr<finenav::core::MapServer<MapT>> FineNavEngine::getMapResource(
    const std::string& name) {
    auto it = std::find_if(map_resources_.begin(), map_resources_.end(),
                           [&](const auto& pair) { return pair.first == name; });
    if (it != map_resources_.end()) {
        return std::any_cast<std::shared_ptr<finenav::core::MapServer<MapT>>>(it->second);
    }
    return nullptr;
}

template <typename PlannersT, typename ExecutionPolicy>
std::shared_ptr<finenav::core::ControlLayer<PlannersT, ExecutionPolicy>>
FineNavEngine::createControlLayer(const std::string& name, ExecutionPolicy policy) {
    auto layer = std::make_shared<finenav::core::ControlLayer<PlannersT, ExecutionPolicy>>(
        node_, name, std::move(policy));
    layer->set_robot_state_provider([this]() -> finenav::core::RobotState {
        return localizer_->getRobotState().value_or(finenav::core::RobotState{});
    });
    held_layers_.push_back(layer);
    return layer;
}

template <typename... LayersT>
void FineNavEngine::registerPipeline(const PipelineIO& io,
                                     std::shared_ptr<LayersT>... layers) {
    static_assert(sizeof...(layers) >= 1, "Pipeline must have at least one layer!");
    auto layer_tuple = std::make_tuple(layers...);

    std::get<0>(layer_tuple)->setInputTopic(io.input_topic);

    constexpr auto last_idx = sizeof...(layers) - 1;
    std::get<last_idx>(layer_tuple)->setOutputTopic(io.output_topic);

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
    RCLCPP_WARN_STREAM(node_->get_logger(),
        "Connecting " << first->getName() << " → " << second->getName()
        << " via topic: " << internal_topic);
    first->setOutputTopic(internal_topic);
    second->setInputTopic(internal_topic);
    wire_layers(second, rest...);
}

}  // namespace finenav
