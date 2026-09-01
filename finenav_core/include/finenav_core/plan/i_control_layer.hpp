// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "finenav_core/map/i_map_resource.hpp"

namespace finenav::core {

class IControlLayer {
public:
    virtual ~IControlLayer() = default;
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
    explicit IControlLayer(std::string name) : name_(std::move(name)) {}
    std::string name_;
};

}  // namespace finenav::core
