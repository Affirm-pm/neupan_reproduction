// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "dummy_map.hpp"
#include "dummy_planner.hpp"

namespace finenav::example {

/**
 * @brief SimpleMapView — the bridge between DummyMap and DummyPlanner::IMapView.
 *
 * This is the ONE artifact that knows both concrete types.  It is written once
 * by the navigation-logic author and registered via FINENAV_REGISTER_MAPVIEW;
 * the end user then assembles the ControlLayer by NAME without ever seeing
 * either DummyMap or DummyPlanner.
 *
 * The constructor convention (ViewT(const Map1&, const Map2&, ...)) is what the
 * framework uses to auto-build the view from the maps it resolved at assembly
 * time — the author only declares the map types in the registration macro.
 */
class SimpleMapView : public DummyPlanner::IMapView {
public:
    explicit SimpleMapView(const DummyMap& map) : map_(map) {}

    bool isWalkable(const finenav::core::Position3D& p) const override {
        return !map_.isOccupied(p);
    }

private:
    const DummyMap& map_;
};

}  // namespace finenav::example
