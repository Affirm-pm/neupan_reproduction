// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "finenav_core/map/i_map_resource.hpp"

namespace finenav::core {

/**
 * @brief IPipelineNode — the type-erased base of every pipeline stage.
 *
 * This is the non-template interface the engine and registry use to hold and
 * wire heterogeneous PipelineNode<PlannerT> instances (whose concrete PlannerT
 * lives inside a plugin DSO).  It carries only the shape-independent operations:
 * topic wiring and MapView attachment.
 */
class IPipelineNode {
public:
    virtual ~IPipelineNode() = default;
    virtual void connectFrom(const std::string& parent_topic) = 0;

    /**
     * @brief Attach a pre-built MapView plus the map resources it reads.
     *
     * The view is a type-erased IMapView (already upcast to PlannerT::IMapView).
     * @p maps are the MapServers backing it; the layer read-locks each one for
     * the duration of every plan() call.
     */
    virtual void attachMapView(
        std::shared_ptr<void> view,
        std::vector<std::shared_ptr<IMapResource>> maps) = 0;

    std::string layerName() const { return name_; }

protected:
    explicit IPipelineNode(std::string name) : name_(std::move(name)) {}
    std::string name_;
};

}  // namespace finenav::core
