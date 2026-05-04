// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "finenav_core/plan/control_layer.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_core/nav_state_manager.hpp"
#include "finenav_localizer/finenav_localizer.hpp"
#include "finenav_navigator/navigator.hpp"

namespace finenav {

class FineNavEngine {
public:

    explicit FineNavEngine(const rclcpp::Node::SharedPtr& node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "Initializing...");

        localizer_ = std::make_unique<FineNavLocalizer>(node_);
        RCLCPP_INFO(node_->get_logger(), "Localizer ready.");

        navigator_ = std::make_unique<Navigator>(node_);
        RCLCPP_INFO(node_->get_logger(), "Navigator ready.");

        nav_state_manager_ = std::make_unique<NavStateManager>(node_, *localizer_);
        RCLCPP_INFO(node_->get_logger(), "NavStateManager ready.");

        RCLCPP_INFO(node_->get_logger(), "All singletons initialized.");
    }

    // 禁止复制，允许移动
    FineNavEngine(const FineNavEngine&) = delete;
    FineNavEngine& operator=(const FineNavEngine&) = delete;
    FineNavEngine(FineNavEngine&&) = default;
    FineNavEngine& operator=(FineNavEngine&&) = default;
    ~FineNavEngine() = default;

    const rclcpp::Node::SharedPtr& getNode() const { return node_; }
    FineNavLocalizer* getLocalizer() { return localizer_.get(); }
    Navigator* getNavigator() { return navigator_.get(); }
    NavStateManager* getNavStateManager() { return nav_state_manager_.get(); }


    template <typename MapT>
    [[nodiscard("Map resource was created but the handle was lost!")]]
    std::shared_ptr<MapServer<MapT>> createMapResource(
        const std::string& name,
        double update_rate_hz     = 10.0,
        bool enable_shift_window  = false,
        double shift_rate_hz      = 50.0)
    {
        auto it = std::find_if(map_resources_.begin(), map_resources_.end(),
                               [&](const auto& pair) { return pair.first == name; });
        if (it == map_resources_.end()) {
            std::shared_ptr<MapServer<MapT>> new_map;
            if (enable_shift_window) {
                new_map = std::make_shared<MapServer<MapT>>(
                    node_, name, update_rate_hz, localizer_.get(), shift_rate_hz);
            } else {
                new_map = std::make_shared<MapServer<MapT>>(node_, name, update_rate_hz);
            }
            map_resources_.emplace_back(name, new_map);
            return new_map;
        }
        return std::any_cast<std::shared_ptr<MapServer<MapT>>>(it->second);
    }

    template <typename MapT>
    [[nodiscard("Map resource was created but the handle was lost!")]]
    std::shared_ptr<MapServer<MapT>> getMapResource(const std::string& name) {
        auto it = std::find_if(map_resources_.begin(), map_resources_.end(),
                               [&](const auto& pair) { return pair.first == name; });
        if (it != map_resources_.end()) {
            return std::any_cast<std::shared_ptr<MapServer<MapT>>>(it->second);
        }
        return nullptr;
    }


    /**
     * @brief 创建 ControlLayer 并自动注入框架回调：
     *   - robot_state_provider : 从 Localizer 读取最新位姿/速度
     *   - should_stop_predicate: 查询 NavStateManager 决定 TrackingPolicy 是否退出
     *
     * 用户仍可在返回的 layer 上调用 set_on_post_plan() 注册应用级输出逻辑。
     */
    template <typename PlannersT, typename ExecutionPolicy = SingleShotPolicy>
    [[nodiscard("ControlLayer was created but the handle was lost!")]]
    std::shared_ptr<ControlLayer<PlannersT, ExecutionPolicy>> createControlLayer(
        const std::string& name,
        ExecutionPolicy policy = ExecutionPolicy{})
    {
        auto layer = std::make_shared<ControlLayer<PlannersT, ExecutionPolicy>>(
            node_, name, std::move(policy));

        // Auto-inject: robot state from Localizer
        layer->set_robot_state_provider([this]() -> finenav_msgs::msg::RobotState {
            const auto& state = localizer_->getState();
            finenav_msgs::msg::RobotState rs;
            rs.pose.position.x = state.p.x();
            rs.pose.position.y = state.p.y();
            rs.pose.position.z = state.p.z();
            const auto q = state.R.unit_quaternion();
            rs.pose.orientation.w = q.w();
            rs.pose.orientation.x = q.x();
            rs.pose.orientation.y = q.y();
            rs.pose.orientation.z = q.z();
            // linear velocity in body frame: v_body = R^{-1} * v_world
            const Eigen::Vector3d v_body = state.R.inverse() * state.v;
            rs.twist.linear.x  = v_body.x();
            rs.twist.linear.y  = v_body.y();
            rs.twist.linear.z  = v_body.z();
            rs.twist.angular.x = state.omega.x();
            rs.twist.angular.y = state.omega.y();
            rs.twist.angular.z = state.omega.z();
            return rs;
        });

        // Auto-inject: stop signal from NavStateManager
        layer->set_should_stop_predicate([this]() {
            return nav_state_manager_->shouldStop();
        });

        held_layers_.push_back(layer);
        return layer;
    }


    struct PipelineIO {
        std::string input_topic;
        std::string output_topic;
    };

    template <typename... LayersT>
    void registerPipeline(const PipelineIO& io, std::shared_ptr<LayersT>... layers) {
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

private:

    template <typename T>
    void wire_layers(std::shared_ptr<T>) {}

    template <typename FirstT, typename SecondT, typename... RestT>
    void wire_layers(std::shared_ptr<FirstT> first,
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

    rclcpp::Node::SharedPtr node_;

    // ── Engine-level singletons ───────────────────────────────────────────────
    std::unique_ptr<FineNavLocalizer>    localizer_;
    std::unique_ptr<Navigator>           navigator_;   // owns pnode_ + NodeThread internally
    std::unique_ptr<NavStateManager>     nav_state_manager_;

    // ── Held resources ────────────────────────────────────────────────────────
    std::vector<std::pair<std::string, std::any>> map_resources_;
    std::vector<std::shared_ptr<void>>            held_layers_;
};

} // namespace finenav
