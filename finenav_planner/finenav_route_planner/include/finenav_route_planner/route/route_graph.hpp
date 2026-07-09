// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace route {
// TODO: 目前只支持2D，后续应该直接推广成3D版本

// =============================================================
// Graph data types
// =============================================================

/// A node in the route graph — a named waypoint in world coordinates.
struct Node {
    unsigned int id    = 0;
    double       x     = 0.0;   ///< world frame, metres
    double       y     = 0.0;
    std::string  frame;          ///< semantic name (e.g. "entrance_A"); empty if unset
};

/// One directed edge in the adjacency list.
struct Neighbor {
    size_t to_index = 0;         ///< index into Graph::nodes
    double cost     = 0.0;       ///< traversal cost in metres (or fixed override)

    // Dense polyline [x, y]: first = start node centre, last = end node centre.
    // Intermediate points come from the GeoJSON LineString geometry.
    std::vector<std::pair<double, double>> polyline;
};

/// Adjacency-list route graph loaded from GeoJSON.
struct Graph {
    std::vector<Node>                        nodes;
    std::vector<std::vector<Neighbor>>       adj;       ///< adj[i] = outgoing edges of nodes[i]
    std::unordered_map<unsigned int, size_t> id_index;  ///< node id → index in nodes

    size_t indexOf(unsigned int node_id) const {
        auto it = id_index.find(node_id);
        return (it != id_index.end()) ? it->second : SIZE_MAX;
    }

    /// Find index by semantic frame name; SIZE_MAX if not found.
    size_t indexOfFrame(const std::string& frame_name) const {
        for (size_t i = 0; i < nodes.size(); ++i)
            if (nodes[i].frame == frame_name) return i;
        return SIZE_MAX;
    }
};

// =============================================================
// GeoJSON loader
//
// Expected FeatureCollection schema:
//
//   Node  — geometry.type = "Point"
//            properties: { id: uint, frame?: string }
//            geometry.coordinates: [x, y]
//
//   Edge  — geometry.type = "LineString" | "MultiLineString"
//            properties: { id: uint, startid: uint, endid: uint,
//                          cost?: float, overridable?: bool }
//            geometry.coordinates: [[x,y], ...]
//
// If overridable == false and cost > 0, that cost is used directly.
// Otherwise the edge cost equals the Euclidean length of the polyline.
//
// Returns nullopt on any parse or schema error.
// =============================================================

std::optional<Graph> loadFromJson(const std::string& geojson);

}  // namespace route

