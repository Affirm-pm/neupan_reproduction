// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/map/i_map_resource.hpp"

namespace finenav::core {

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
     * The view is a type-erased IWorldView (already upcast to PlannerT::IWorldView).
     * @p maps are the MapServers backing it; the layer read-locks each one for
     * the duration of every plan() call.
     */
    virtual void attachWorldView(
        std::shared_ptr<void> view,
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
