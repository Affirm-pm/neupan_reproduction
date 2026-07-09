// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_map.hpp"

namespace finenav::example {

DummyMap::DummyMap()
    : bounds_(finenav::core::Position3D(-50, -50, -5),
              finenav::core::Position3D( 50,  50,  5)) {}

void DummyMap::configure(const ConfigType& cfg) {
    window_size_ = cfg.window_size;
}

bool DummyMap::isInside(const finenav::core::Position3D& p) const {
    return bounds_.contains(p);
}

finenav::core::Region3D DummyMap::getWindowBounds() const { return bounds_; }

finenav::core::Position3D DummyMap::getWindowCenter() const {
    return bounds_.center();
}

bool DummyMap::isOccupied(const finenav::core::Position3D& p) const {
    return false;   // DummyMap never reports occupancy
}

}  // namespace finenav::example
