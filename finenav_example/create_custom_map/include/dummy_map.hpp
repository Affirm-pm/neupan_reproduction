// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/algo_configurator.hpp"
#include "finenav_core/math_def.hpp"

#include "finenav_example/dummy_map_params.hpp"

namespace finenav::example {

class DummyMap {
public:
    using ConfigType = dummy_map::Params;
    using DataType   = int;

    DummyMap();

    void configure(const ConfigType& cfg);

    bool isInside(const finenav::core::Position3D& p) const;
    finenav::core::Region3D   getWindowBounds()  const;
    finenav::core::Position3D getWindowCenter()  const;

    bool isOccupied(const finenav::core::Position3D& p) const;

private:
    double window_size_ = 50.0;
    finenav::core::Region3D bounds_;
};

}  // namespace finenav::example

FINENAV_REGISTER_ALGO_CONFIG(finenav::example::DummyMap, dummy_map)
