// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "finenav_core/plan/control_layer.hpp"

namespace finenav {

class FineNavManager {
public:

    FineNavManager(const rclcpp::Node::SharedPtr& node) : node_(node) {
        RCLCPP_INFO(node_->get_logger(), "FinenavManager initialized"); // TODO：搞一个基于ROS2 child logger 机制的日志系统
    }

    // 禁止复制，允许移动
    FineNavManager(const FineNavManager&) = delete;
    FineNavManager& operator=(const FineNavManager&) = delete;

    FineNavManager(FineNavManager&& other) = default;
    FineNavManager& operator=(FineNavManager&& other) = default;

    ~FineNavManager() {}

    const rclcpp::Node::SharedPtr& getNode() const {
        return node_;
    }

    template <typename MapT> // TODO: MapConcept
    [[nodiscard("Map resource was created but the handle was lost!")]]
    std::shared_ptr<MapT> createMapResource(const std::string& name) {
        auto it = std::find_if(map_resources_.begin(), map_resources_.end(),
                                            [&](const auto& pair) { return pair.first == name; });
        if (it == map_resources_.end()) {
            auto new_map = std::make_shared<MapT>(); // TODO: 参数管理可以被一个 MapServer 包裹
            map_resources_.emplace_back(name, new_map);
            return new_map;
        }
        return std::any_cast<std::shared_ptr<MapT>>(it->second);
    }

    template <typename MapT>
    [[nodiscard("Map resource was created but the handle was lost!")]]
    std::shared_ptr<MapT> getMapResource(const std::string& name) {
        auto it = std::find_if(map_resources_.begin(), map_resources_.end(),
                                            [&](const auto& pair) { return pair.first == name; });
        if (it != map_resources_.end()) {
            return std::any_cast<std::shared_ptr<MapT>>(it->second);
        }
        return nullptr;
    }

    template <typename PlannersT>
    [[nodiscard("ControlLayer was created but the handle was lost!")]]
    std::shared_ptr<ControlLayer<PlannersT>> createControlLayer(const std::string& name) {
        auto layer = std::make_shared<ControlLayer<PlannersT>>(node_,  name);
        held_layers_.push_back(layer);  // TODO: prevent same name
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

    // 递归终止条件 (少于2个Layer不接线)
    template <typename T>
    void wire_layers(std::shared_ptr<T>) {}

    template <typename FirstT, typename SecondT, typename... RestT>
    void wire_layers(std::shared_ptr<FirstT> first,
                          std::shared_ptr<SecondT> second,
                          std::shared_ptr<RestT>... rest) {
        std::string internal_topic = first->getName() + "_to_" + second->getName();
        RCLCPP_WARN_STREAM(node_->get_logger(), "Connecting " << first->getName() << " output to " << second->getName() << " input via topic: " << internal_topic);

        first->setOutputTopic(internal_topic);
        second->setInputTopic(internal_topic);

        wire_layers(second, rest...);
    }


    rclcpp::Node::SharedPtr node_;
    std::vector<std::pair<std::string, std::any>> map_resources_; // TODO：map_resources_ 或许也只需要保活，不需要一个名字来标识？？ 感觉不靠谱
    std::vector<std::shared_ptr<void>> held_layers_; // 持有 ControlLayer 实例，防止被销毁
    std::vector<std::shared_ptr<void>> held_param_handles_;

    // TODO: CurrentRobotState = 运动状态 + 导航任务状态； TF 时间戳相关细节，可参考 nav2_planner

};
}
