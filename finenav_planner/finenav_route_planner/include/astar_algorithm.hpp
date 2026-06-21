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

#include "grid_search_utils.hpp"   // Connectivity, getNeighbors, calculateStepCost,
                                    // cellDistance, reconstructPathImpl
#include "heuristic.hpp"

namespace astar {

// =============================================================
// AstarNode  —  internal grid search node, always Vec3-based
// =============================================================

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

// =============================================================
// AStarAlgorithm
//
// A plain uniform-grid A* engine.
//
// Neighbourhood and shared geometry utilities come from
// grid_search_utils.hpp (shared with BfsAlgorithm).
// The only A*-specific logic here is the priority-queue and
// the g/h/f node bookkeeping.
// =============================================================

class AStarAlgorithm {
public:

    using Connectivity = astar::Connectivity;   // re-export from grid_search_utils

    using IsWalkableFn = std::function<bool(const Vec3&)>;
    using GetCostFn    = std::function<double(const Vec3&)>;
    using IsGoalFn     = std::function<bool(const Vec3&)>;

    /// Optional filter for findPathToAny — when a goal cell is found,
    /// return true to accept (stop) or false to reject (continue searching).
    /// nullptr means accept all goals (default behaviour).
    using GoalFilterFn = std::function<bool(const Vec3&)>;

    AStarAlgorithm(IsWalkableFn is_walkable, GetCostFn get_cost);

    // ── setters ───────────────────────────────────────────────
    void setConnectivity(Connectivity c)  { connectivity_      = c; }
    void setHeuristic(Heuristic h)        { heuristic_         = h; }
    void setGoalTolerance(double t)       { goal_tolerance_    = t; }
    void setMaxIterations(int m)          { max_iterations_    = m; }
    void setMaxPlanningTime(double t)     { max_planning_time_ = t; }
    void setMovementWeight(double w)      { movement_weight_   = w; }
    void setGreedyWeight(double w)        { greedy_weight_     = w; }

    // ── main searches ─────────────────────────────────────────

    /// Standard A* to a fixed goal cell.
    std::optional<std::vector<Vec3>> findPath(const Vec3& start, const Vec3& goal);

    /// Dijkstra (h=0) search to *any* cell accepted by is_goal.
    /// Useful for nearest-neighbour queries where the goal set is dynamic.
    /// The configured heuristic_ is ignored; h is forced to 0.
    ///
    /// When accept_goal is provided: a goal cell that passes is_goal but
    /// fails accept_goal causes the search to continue (recorded as fallback).
    /// If no goal passes accept_goal, the best fallback is returned.
    std::optional<std::vector<Vec3>> findPathToAny(
        const Vec3& start,
        IsGoalFn    is_goal,
        GoalFilterFn accept_goal = nullptr);

private:
    std::vector<Vec3> reconstructPath(const Vec3& goal) const;

    // ── injected queries ──────────────────────────────────────
    IsWalkableFn is_walkable_;
    GetCostFn    get_cost_;

    // ── parameters ────────────────────────────────────────────
    Connectivity connectivity_      = Connectivity::N8;
    Heuristic    heuristic_         = Heuristic::Euclidean;
    double       goal_tolerance_    = 0.25;
    int          max_iterations_    = 1000000;
    double       max_planning_time_ = 5.0;
    double       movement_weight_   = 1.0;
    double       greedy_weight_     = 1.001;

    // ── search state (reset each findPath call) ───────────────
    std::priority_queue<AstarNode, std::vector<AstarNode>, AstarNodeCompare> open_set_;
    std::unordered_set<Vec3, Vec3Hash>            visited_;
    std::unordered_map<Vec3, AstarNode, Vec3Hash> came_from_;
};

} // namespace astar

