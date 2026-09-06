// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_route_planner/imap_view.hpp"
#include "finenav_route_planner/route/route_graph.hpp"

namespace finenav {

// MapView for RoutePathSearch: adds the topological route graph (non-map data,
// e.g. GeoJSON) on top of the occupancy-map queries inherited from
// astar::IMapView (isWalkable / getCost / getStepSize).
class IRouteMapView : public astar::IMapView {
public:
    virtual ~IRouteMapView() = default;

    /// The route graph (topological network), or nullptr when not yet loaded.
    virtual const route::Graph* getGraph() const = 0;
};

}  // namespace finenav
