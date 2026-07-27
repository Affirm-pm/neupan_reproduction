// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <cstddef>
#include <functional>

#include <Eigen/Core>

namespace finenav {

using Position = Eigen::Vector3d;  // from grid_map_math.hpp

/**
 * @brief Voxel cell data for TemporalVoxelMap.
 *
 * Each voxel records its observed height and whether it represents static
 * terrain.  The timestamp tracks when the voxel was last observed, used
 * by pruneExpiredCells() to remove stale dynamic obstacles.
 */
struct Voxel {
    float z_height = NAN;
    bool is_static = false;
    std::chrono::steady_clock::time_point timestamp{};
};

// ── Hash / Equal for unordered_set<Position> ───────────────────────────

struct PositionHash {
    std::size_t operator()(const Position& p) const noexcept {
        auto h = [](double v) { return std::hash<double>{}(v); };
        std::size_t seed = 0;
        seed ^= h(p.x()) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        seed ^= h(p.y()) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        seed ^= h(p.z()) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct PositionEqual {
    bool operator()(const Position& a, const Position& b) const noexcept {
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

using DynamicPosSet = std::unordered_set<Position, PositionHash, PositionEqual>;

}  // namespace finenav
