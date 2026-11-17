// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// Grid connectivity, geometric costs and path reconstruction.

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "finenav_route_planner/common/vec3.hpp"

namespace astar {

enum class Connectivity { N4, N8, N6, N18, N26 };

/// Returns the grid neighbours of `pos` under the given connectivity.
inline std::vector<Vec3> getNeighbors(const Vec3& pos, Connectivity c)
{
    static const std::array<Vec3, 4> ortho2d = {{
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0}
    }};
    static const std::array<Vec3, 4> diag2d = {{
        {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0}
    }};
    static const std::array<Vec3, 6> face3d = {{
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}
    }};
    static const std::array<Vec3, 12> edge3d = {{
        {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0},
        {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
        {0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1}
    }};
    static const std::array<Vec3, 8> corner3d = {{
        {1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
        {-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}
    }};

    std::vector<Vec3> result;
    switch (c)
    {
        case Connectivity::N4:
            for (const auto& o : ortho2d)  result.push_back(pos + o);
            break;
        case Connectivity::N8:
            for (const auto& o : ortho2d)  result.push_back(pos + o);
            for (const auto& o : diag2d)   result.push_back(pos + o);
            break;
        case Connectivity::N6:
            for (const auto& o : face3d)   result.push_back(pos + o);
            break;
        case Connectivity::N18:
            for (const auto& o : face3d)   result.push_back(pos + o);
            for (const auto& o : edge3d)   result.push_back(pos + o);
            break;
        case Connectivity::N26:
            for (const auto& o : face3d)   result.push_back(pos + o);
            for (const auto& o : edge3d)   result.push_back(pos + o);
            for (const auto& o : corner3d) result.push_back(pos + o);
            break;
    }
    return result;
}

/// Step cost purely from geometry: 1 axis → 1.0,  2 axes → √2,  3 axes → √3
inline double calculateStepCost(const Vec3& from, const Vec3& to) noexcept
{
    int nd = (from.x != to.x) + (from.y != to.y) + (from.z != to.z);
    if (nd <= 1) return 1.0;
    if (nd == 2) return std::sqrt(2.0);
    return std::sqrt(3.0);
}

/// Euclidean distance between two cells.
inline double cellDistance(const Vec3& a, const Vec3& b) noexcept
{
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/// Reconstruct a path through parent_map using get_parent(const NodeT&) -> Vec3.
/// The start cell is its own parent.
template<typename NodeT, typename GetParentFn>
inline std::vector<Vec3> reconstructPathImpl(
    const Vec3& goal,
    const std::unordered_map<Vec3, NodeT, Vec3Hash>& parent_map,
    GetParentFn get_parent)
{
    std::vector<Vec3> path;
    Vec3 cur = goal;
    while (true) {
        path.push_back(cur);
        const auto& node   = parent_map.at(cur);
        const Vec3  parent = get_parent(node);
        if (cur == parent) break;   // self-sentinel == start
        cur = parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace astar

