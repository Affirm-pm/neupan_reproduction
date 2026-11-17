// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <functional>
#include <optional>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <cmath>

#include "finenav_route_planner/common/grid_search_utils.hpp"
#include "finenav_route_planner/common/heuristic.hpp"

namespace astar {

// Grid search node with accumulated cost, heuristic and parent cell.
struct AstarNode {
    Vec3   pos;
    double g = std::numeric_limits<double>::max();
    double h = 0.0;
    double f = std::numeric_limits<double>::max();
    Vec3   parent{};   // sentinel: parent == pos means this is the start

    AstarNode() = default;
    explicit AstarNode(const Vec3& p) : pos(p) {}
    void updateF() { f = g + h; }
};

struct AstarNodeCompare {
    bool operator()(const AstarNode& a, const AstarNode& b) const
    { return a.f > b.f; }
};

/// Uniform-grid A* search; Heuristic::Zero selects Dijkstra search.
class AStarAlgorithm {
public:

    using Connectivity = astar::Connectivity;   // re-export from grid_search_utils

    using IsWalkableFn = std::function<bool(const Vec3&)>;
    using GetCostFn    = std::function<double(const Vec3&)>;
    using IsGoalFn     = std::function<bool(const Vec3&)>;

    /// Accepts a goal or keeps searching; an empty filter accepts every goal.
    using GoalFilterFn = std::function<bool(const Vec3&)>;

    AStarAlgorithm(IsWalkableFn is_walkable, GetCostFn get_cost);

    // setters
    void setConnectivity(Connectivity c)  { connectivity_      = c; }
    void setHeuristic(Heuristic h)        { heuristic_         = h; }
    void setGoalTolerance(double t)       { goal_tolerance_    = t; }
    void setMaxIterations(int m)          { max_iterations_    = m; }
    void setMaxPlanningTime(double t)     { max_planning_time_ = t; }
    void setMovementWeight(double w)      { movement_weight_   = w; }
    void setGreedyWeight(double w)        { greedy_weight_     = w; }

    // main searches
    /// Standard A* to a fixed goal cell.
    std::optional<std::vector<Vec3>> findPath(const Vec3& start, const Vec3& goal);

    /// Searches is_goal matches in accumulated-cost order with h = 0.
    /// Returns the lowest-cost rejected goal if no goal passes accept_goal.
    std::optional<std::vector<Vec3>> findPathToAny(
        const Vec3& start,
        IsGoalFn    is_goal,
        GoalFilterFn accept_goal = nullptr);

private:
    std::vector<Vec3> reconstructPath(const Vec3& goal) const;

    // injected queries
    IsWalkableFn is_walkable_;
    GetCostFn    get_cost_;

    // parameters
    Connectivity connectivity_      = Connectivity::N8;
    Heuristic    heuristic_         = Heuristic::Euclidean;
    double       goal_tolerance_    = 0.25;
    int          max_iterations_    = 1000000;
    double       max_planning_time_ = 5.0;
    double       movement_weight_   = 1.0;
    double       greedy_weight_     = 1.001;

    // Search state, reset before each search.
    std::priority_queue<AstarNode, std::vector<AstarNode>, AstarNodeCompare> open_set_;
    std::unordered_set<Vec3, Vec3Hash>            visited_;
    std::unordered_map<Vec3, AstarNode, Vec3Hash> came_from_;
};

} // namespace astar

