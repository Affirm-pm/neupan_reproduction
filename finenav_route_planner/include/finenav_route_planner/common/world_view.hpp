// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include "finenav_common/geometry_types.hpp"

namespace astar {
using finenav::common::Position3D;
using finenav::common::Region3D;

/// World-coordinate map queries for grid planners.
/// Positions use WORLD_FIXED_FRAME in metres; the grid is anchored at (0, 0, 0).
class IWorldView {
public:
    virtual ~IWorldView() = default;

    /// Whether the robot can traverse world point p.
    virtual bool isWalkable(const finenav::common::Position3D& p) const = 0;

    /// Nonnegative additional traversal cost at p.
    /// Edge weight = step_cost * (movement_weight + getCost(p)).
    virtual double getCost(const finenav::common::Position3D& p) const = 0;

    /// Grid spacing in metres; must be finite and positive.
    virtual double getStepSize() const = 0;
};

} // namespace astar
