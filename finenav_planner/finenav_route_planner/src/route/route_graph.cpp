// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "route/route_graph.hpp"

#include <cmath>
#include <nlohmann/json.hpp>

namespace route {

namespace {

using Json = nlohmann::json;

double polylineLength(const std::vector<std::pair<double, double>>& pts)
{
    double len = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
        double dx = pts[i].first  - pts[i - 1].first;
        double dy = pts[i].second - pts[i - 1].second;
        len += std::hypot(dx, dy);
    }
    return len;
}

std::vector<std::pair<double, double>> collectPolyline(
    const Json& geometry, const std::string& gtype)
{
    std::vector<std::pair<double, double>> pts;
    const auto& coords = geometry["coordinates"];

    if (gtype == "LineString") {
        pts.reserve(coords.size());
        for (const auto& p : coords)
            pts.emplace_back(p[0].get<double>(), p[1].get<double>());
    } else {                         // MultiLineString
        for (const auto& line : coords)
            for (const auto& p : line)
                pts.emplace_back(p[0].get<double>(), p[1].get<double>());
    }
    return pts;
}

}  // anonymous namespace

std::optional<Graph> loadFromJson(const std::string& geojson)
{
    Json j;
    try { j = Json::parse(geojson); }
    catch (...) { return std::nullopt; }

    if (!j.contains("features") || !j["features"].is_array())
        return std::nullopt;

    Graph g;

    // ── Pass 1: nodes ──────────────────────────────────────────────────────
    for (const auto& feat : j["features"]) {
        if (!feat.contains("geometry")) continue;
        if (feat["geometry"]["type"] != "Point") continue;

        const auto& props  = feat["properties"];
        const auto& coords = feat["geometry"]["coordinates"];
        if (!coords.is_array() || coords.size() < 2) return std::nullopt;
        if (!props.contains("id")) return std::nullopt;

        Node node;
        node.id = props["id"].get<unsigned int>();
        node.x  = coords[0].get<double>();
        node.y  = coords[1].get<double>();
        if (props.contains("frame") && props["frame"].is_string())
            node.frame = props["frame"].get<std::string>();

        g.id_index[node.id] = g.nodes.size();
        g.nodes.push_back(std::move(node));
    }

    if (g.nodes.empty()) return std::nullopt;
    g.adj.resize(g.nodes.size());

    // ── Pass 2: edges ──────────────────────────────────────────────────────
    for (const auto& feat : j["features"]) {
        if (!feat.contains("geometry")) continue;
        const std::string gtype = feat["geometry"]["type"];
        if (gtype != "LineString" && gtype != "MultiLineString") continue;

        const auto& props = feat["properties"];
        if (!props.contains("startid") || !props.contains("endid"))
            return std::nullopt;

        const size_t si = g.indexOf(props["startid"].get<unsigned int>());
        const size_t ei = g.indexOf(props["endid"].get<unsigned int>());
        if (si == SIZE_MAX || ei == SIZE_MAX) return std::nullopt;

        auto polyline = collectPolyline(feat["geometry"], gtype);
        if (polyline.empty()) {
            polyline.emplace_back(g.nodes[si].x, g.nodes[si].y);
            polyline.emplace_back(g.nodes[ei].x, g.nodes[ei].y);
        }

        // Use fixed cost only when explicitly set as non-overridable.
        const bool fixed_cost =
            props.contains("cost") &&
            props.contains("overridable") &&
            !props["overridable"].get<bool>() &&
            props["cost"].get<double>() > 0.0;

        const double cost = fixed_cost
            ? props["cost"].get<double>()
            : polylineLength(polyline);

        g.adj[si].push_back({ei, cost, std::move(polyline)});
    }

    return g;
}

}  // namespace route

