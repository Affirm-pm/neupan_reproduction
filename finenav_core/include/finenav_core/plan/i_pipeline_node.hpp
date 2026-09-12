// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/common/i_param_pollable.hpp"
#include "finenav_core/map/i_map_resource.hpp"

namespace finenav::core {

/**
 * @brief The product of building a WorldView — everything the framework needs
 *        to drive it for the rest of the layer's life.
 *
 * All three members are type-erased because the builder runs where ViewT and
 * PlannerT are concrete (the registration macro), while the consumer —
 * PipelineNode — only ever sees `PlannerT::IWorldView`.
 *
 * `prepare` has to be an erased invoker rather than a virtual on the view
 * interface: by the time planOnce() holds the view, its static type is
 * `PlannerT::IWorldView`, so `prepare()` could neither be detected nor called
 * from there.  The invoker is generated where ViewT is still concrete and casts
 * the erased context back to `const PlanningContext<PlannerT>*`.
 */
struct BuiltWorldView {
    /// Type-erased `void(const PlanningContext<PlannerT>*)`.
    using PrepareFn = std::function<void(const void*)>;

    std::shared_ptr<void>           view;     ///< already upcast to PlannerT::IWorldView
    std::shared_ptr<IParamPollable> params;   ///< param channel (+ visualizer lifetime)
    PrepareFn                       prepare;  ///< WorldView::prepare() invoker
};

/**
 * @brief IPipelineNode — the type-erased base of every pipeline stage.
 *
 * This is the non-template interface the engine and registry use to hold and
 * wire heterogeneous PipelineNode<PlannerT> instances (whose concrete PlannerT
 * lives inside a plugin DSO).  It carries only the shape-independent operations:
 * topic wiring and WorldView attachment.
 */
class IPipelineNode {
public:
    virtual ~IPipelineNode() = default;

    /// Register a possible predecessor for this stage, given the upstream
    /// **layer name** (not a topic string).  The stage subscribes to that
    /// upstream layer's feedback topic and consumes it as the reference input.
    /// Multiple predecessors (fan-out) may be registered; only the upstream of
    /// the ACTIVE branch publishes, so the stage sees at most one live ref.
    virtual void connectFrom(const std::string& parent_name) = 0;

    /**
     * @brief Attach a pre-built WorldView plus the map resources it reads.
     *
     * @p built carries the type-erased view (already upcast to
     * PlannerT::IWorldView), its parameter pollable and its prepare() invoker.
     * @p maps are the MapServers backing it; the layer read-locks each one for
     * the duration of every plan() call, so prepare() and plan() both see one
     * consistent map revision.
     */
    virtual void attachWorldView(
        BuiltWorldView built,
        std::vector<std::shared_ptr<IMapResource>> maps) = 0;

    /// The stage's private sub-node (pnode).  Handed to WorldView build so a
    /// subscribing WorldView shares the layer's node/executor.
    virtual rclcpp::Node::SharedPtr getPrivateNode() const = 0;

    std::string layerName() const { return name_; }

protected:
    explicit IPipelineNode(std::string name) : name_(std::move(name)) {}
    std::string name_;
};

}  // namespace finenav::core
