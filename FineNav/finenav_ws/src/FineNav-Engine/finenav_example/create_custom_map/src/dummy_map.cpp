// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_map.hpp"

#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {

DummyMap::DummyMap()
    : bounds_{finenav::common::Position3D{-50, -50, -5},
              finenav::common::Position3D{ 50,  50,  5}} {}

void DummyMap::configure(const ConfigType& cfg) {
    RCLCPP_INFO(logger_, "DummyMap configured: window_size=%.1f", cfg.window_size);
    window_size_ = cfg.window_size;
}

bool DummyMap::isInside(const finenav::common::Position3D& p) const {
    return contains(bounds_, p);
}

finenav::common::Region3D DummyMap::getWindowBounds() const { return bounds_; }

finenav::common::Position3D DummyMap::getWindowCenter() const {
    return center(bounds_);
}

}  // namespace finenav::example

// One-line registration: parameter wiring + plugin self-registration.
// "dummy_map" must match the top-level key in dummy_map.yaml.
FINENAV_REGISTER_MAP(finenav::example::DummyMap, dummy_map)
