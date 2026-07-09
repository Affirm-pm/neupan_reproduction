// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include "finenav_core/math_def.hpp"   // Position3D

namespace astar {
using finenav::core::Position3D;
using finenav::core::Region3D;

// =============================================================
// IMapView  —  user-facing world-coordinate map interface
//
// Single unified interface for both AstarPathSearch and BfsPathSearch.
// BFS planners may ignore getCost() (return 0.0 is fine).
//
// Implement this to connect your map to the grid planners.
// You do not need to know anything about Vec3, grid cells, or
// search algorithm internals.
//
// ── Two things you provide ────────────────────────────────────
//
//   1. World-coordinate queries
//      isWalkable / getCost
//      All inputs are in world space (metres, WORLD_FIXED_FRAME).
//
//   2. Search step size  (getStepSize)
//      "When the planner moves one cell, how many metres does it travel?"
//
//      OccupancyGrid  →  return info.resolution
//      Voxel map      →  return voxel_size
//      Custom SDF     →  return desired_sampling_interval
//
// Note: In FineNav-Engine every map uses WORLD_FIXED_FRAME, so
// the grid origin is always (0, 0, 0) and you never need to
// specify it.
// =============================================================

class IMapView {
public:
    virtual ~IMapView() = default;

    // ── world-coordinate queries ──────────────────────────────
    /// Can the robot pass through world point p?
    virtual bool isWalkable(const finenav::core::Position3D& p) const = 0;

    /// Extra traversal cost at p (must be >= 0).
    /// A* edge weight = step_cost * (1 + getCost(p)).
    /// Return 0.0 for a uniform-cost map (e.g. when used with BFS).
    virtual double getCost(const finenav::core::Position3D& p) const = 0;

    // ── discretisation hint ───────────────────────────────────
    /// Distance in metres that the planner travels with each grid step.
    virtual double getStepSize() const = 0;
};

} // namespace astar

