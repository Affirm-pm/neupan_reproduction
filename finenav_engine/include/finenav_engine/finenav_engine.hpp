// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <any>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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
 * 1. createMapServer<MapT>(name)        — engine stores the MapServer internally
 * 2. createControlLayer<PlannersT>(name) — engine stores the ControlLayer internally
 * 3. map_server    = getMapServer<MapT>(name)       — retrieve typed handle
 * 4. control_layer = getControlLayer<PlannersT>(name)
 * 5. control_layer->bind_map_view(map_server, ...) — user connects map to layer
 * 6. connectPipeline(p, control_layers...)         — engine wires the pipeline topics
 *
 * The engine owns all handles.  The user retrieves them by name + type at the
 * call site.  FineNavEngine itself is non-template — pass it by pointer.
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
     * @brief Create a named MapServer and store it internally.
     * @tparam MapT  Must satisfy SpatialContainer.
     */
    template <typename MapT>
    void createMapServer(const std::string& name);

    /**
     * @brief Retrieve a previously-created MapServer by name and type.
     * @return nullptr if not found or type mismatch.
     */
    template <typename MapT>
    std::shared_ptr<finenav::core::MapServer<MapT>> getMapServer(const std::string& name);

    /**
     * @brief Remove a MapServer from the engine.  The handle (and map) is
     *        destroyed once all external shared_ptrs to it are released.
     * @return true if a map with the given name existed.
     */
    bool removeMapServer(const std::string& name);

    // ── Factory: layer ────────────────────────────────────────────────────

    /**
     * @brief Create a named ControlLayer and store it internally.
     * @tparam Planners  The planner types (one or more) supported by this layer.
     * @tparam PolicyT   SingleShotPolicy or ContinuousPolicy (deduced from argument).
     */
    template <typename... Planners, typename PolicyT = finenav::core::SingleShotPolicy>
    void createControlLayer(const std::string& name, PolicyT policy = {});

    /**
     * @brief Retrieve a previously-created ControlLayer by name and type.
     * @return nullptr if not found or type mismatch.
     */
    template <typename... Planners>
    std::shared_ptr<finenav::core::ControlLayer<Planners...>> getControlLayer(const std::string& name);

    /**
     * @brief Remove a ControlLayer from the engine.  The handle (and layer) is
     *        destroyed once all external shared_ptrs to it are released.
     * @return true if a layer with the given name existed.
     */
    bool removeControlLayer(const std::string& name);

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

    // ── Handle storage (engine owns all user-facing handles) ───────────────
    std::unordered_map<std::string, std::any> maps_;
    std::unordered_map<std::string, std::any> layers_;

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

    navigator_ = std::make_unique<Navigator>(node_, localizer_.get());
    FINENAV_LOG_INFO(node_->get_logger(), "Navigator ready.");

    FINENAV_LOG_INFO(node_->get_logger(), "All singletons initialized.");
}

// ══════════════════════════════════════════════════════════════════════════════
//  Factory: map
// ══════════════════════════════════════════════════════════════════════════════

template <typename MapT>
void FineNavEngine::createMapServer(const std::string& name) {
    auto server = std::make_shared<finenav::core::MapServer<MapT>>(
        node_, name, localizer_.get());
    maps_[name] = std::any(std::move(server));
}

template <typename MapT>
std::shared_ptr<finenav::core::MapServer<MapT>> FineNavEngine::getMapServer(const std::string& name) {
    auto it = maps_.find(name);
    if (it == maps_.end()) return nullptr;
    try {
        return std::any_cast<std::shared_ptr<finenav::core::MapServer<MapT>>>(it->second);
    } catch (const std::bad_any_cast&) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "getMapServer('{}'): type mismatch — stored MapT differs from requested type.", name);
        return nullptr;
    }
}

inline bool FineNavEngine::removeMapServer(const std::string& name) {
    return maps_.erase(name) > 0;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Factory: layer
// ══════════════════════════════════════════════════════════════════════════════

template <typename... Planners, typename PolicyT>
void FineNavEngine::createControlLayer(const std::string& name, PolicyT policy) {
    auto layer = std::make_shared<finenav::core::ControlLayer<Planners...>>(
        node_, name, std::move(policy), localizer_.get());
    layers_[name] = std::any(std::move(layer));
}

template <typename... Planners>
std::shared_ptr<finenav::core::ControlLayer<Planners...>>
FineNavEngine::getControlLayer(const std::string& name) {
    auto it = layers_.find(name);
    if (it == layers_.end()) return nullptr;
    try {
        return std::any_cast<std::shared_ptr<finenav::core::ControlLayer<Planners...>>>(
            it->second);
    } catch (const std::bad_any_cast&) {
        FINENAV_LOG_ERROR(node_->get_logger(),
            "getControlLayer('{}'): type mismatch — stored planner types differ from requested.", name);
        return nullptr;
    }
}

inline bool FineNavEngine::removeControlLayer(const std::string& name) {
    return layers_.erase(name) > 0;
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
