// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/astar_algorithm.hpp"

#include <algorithm>
#include <chrono>

namespace astar {

// ── constructor ────────────────────────────────────────────────────────────

AStarAlgorithm::AStarAlgorithm(IsWalkableFn is_walkable, GetCostFn get_cost)
    : is_walkable_(std::move(is_walkable)),
      get_cost_(std::move(get_cost))
{}

// ── findPath ───────────────────────────────────────────────────────────────

std::optional<std::vector<Vec3>> AStarAlgorithm::findPath(
    const Vec3& start,
    const Vec3& goal)
{
    if (!is_walkable_(start) || !is_walkable_(goal))
        return std::nullopt;

    open_set_ = {};
    visited_.clear();
    came_from_.clear();

    int  iterations = 0;
    auto start_time = std::chrono::steady_clock::now();

    std::optional<AstarNode> best_goal_node;
    double best_goal_dist = std::numeric_limits<double>::infinity();

    AstarNode start_node(start);
    start_node.g      = 0.0;
    start_node.h      = calculateHeuristic(heuristic_, start, goal);
    start_node.updateF();
    start_node.parent = start;   // self-sentinel

    open_set_.push(start_node);
    came_from_[start] = start_node;

    while (!open_set_.empty())
    {
        if (max_iterations_ >= 0 && iterations++ > max_iterations_)
            break;

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > max_planning_time_)
            break;

        AstarNode current = open_set_.top();
        open_set_.pop();

        if (visited_.contains(current.pos)) continue;
        visited_.insert(current.pos);

        if (current.pos == goal)
            return reconstructPath(goal);

        double goal_dist = cellDistance(current.pos, goal);
        if (goal_dist < best_goal_dist) {
            best_goal_dist = goal_dist;
            best_goal_node = current;
        }

        for (const Vec3& nb_pos : getNeighbors(current.pos, connectivity_))
        {
            if (!is_walkable_(nb_pos))     continue;
            if (visited_.contains(nb_pos)) continue;

            double step_cost   = calculateStepCost(current.pos, nb_pos);
            double map_cost    = get_cost_(nb_pos);
            double tentative_g = current.g + step_cost * (movement_weight_ + map_cost);

            if (!came_from_.contains(nb_pos) ||
                tentative_g < came_from_[nb_pos].g)
            {
                AstarNode nb(nb_pos);
                nb.g      = tentative_g;
                nb.h      = calculateHeuristic(heuristic_, nb_pos, goal) * greedy_weight_;
                nb.updateF();
                nb.parent = current.pos;

                came_from_[nb_pos] = nb;
                open_set_.push(nb);
            }
        }
    }

    if (best_goal_node.has_value() && best_goal_dist <= goal_tolerance_)
        return reconstructPath(best_goal_node->pos);

    return std::nullopt;
}

// ── reconstructPath ───────────────────────────────────────────────────────
// Delegates to the shared template helper in grid_search_utils.hpp.

std::vector<Vec3> AStarAlgorithm::reconstructPath(const Vec3& goal) const
{
    return reconstructPathImpl(
        goal, came_from_,
        [](const AstarNode& n) { return n.parent; });
}

// TODO： 要想一个好一点的方式组织，毕竟它只对bfs模式下生效
// ── findPathToAny ──────────────────────────────────────────────────────────
//
// Dijkstra search (h forced to 0) to any cell matching is_goal.
// The configured heuristic_ is deliberately ignored: without a fixed goal
// we cannot compute a valid admissible heuristic, so we fall back to
// uniform-cost expansion.  This is correct (optimal) and consistent with
// BFS on weighted grids.
//
// When accept_goal is provided: a goal cell that passes is_goal but fails
// accept_goal causes the search to continue (it is recorded as a fallback).
// If no goal passes accept_goal, the best (lowest-g) fallback is returned
// so the caller always gets a result when any goal cell is reachable.

std::optional<std::vector<Vec3>> AStarAlgorithm::findPathToAny(
    const Vec3& start,
    IsGoalFn    is_goal,
    GoalFilterFn accept_goal)
{
    if (!is_walkable_(start))
        return std::nullopt;

    open_set_ = {};
    visited_.clear();
    came_from_.clear();

    int  iterations = 0;
    auto start_time = std::chrono::steady_clock::now();

    AstarNode start_node(start);
    start_node.g      = 0.0;
    start_node.h      = 0.0;   // forced — no fixed goal
    start_node.updateF();
    start_node.parent = start;

    open_set_.push(start_node);
    came_from_[start] = start_node;

    std::optional<Vec3> best_fallback;   // best rejected goal (lowest g)

    while (!open_set_.empty())
    {
        if (max_iterations_ >= 0 && iterations++ > max_iterations_)
            break;

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed > max_planning_time_)
            break;

        AstarNode current = open_set_.top();
        open_set_.pop();

        if (visited_.contains(current.pos)) continue;
        visited_.insert(current.pos);

        if (is_goal(current.pos))
        {
            if (!accept_goal || accept_goal(current.pos))
                return reconstructPath(current.pos);

            // Goal found but rejected — record as fallback, continue search
            if (!best_fallback.has_value())
                best_fallback = current.pos;
        }

        for (const Vec3& nb_pos : getNeighbors(current.pos, connectivity_))
        {
            if (!is_walkable_(nb_pos))     continue;
            if (visited_.contains(nb_pos)) continue;

            double step_cost   = calculateStepCost(current.pos, nb_pos);
            double map_cost    = get_cost_(nb_pos);
            double tentative_g = current.g + step_cost * (movement_weight_ + map_cost);

            if (!came_from_.contains(nb_pos) ||
                tentative_g < came_from_[nb_pos].g)
            {
                AstarNode nb(nb_pos);
                nb.g      = tentative_g;
                nb.h      = 0.0;   // h=0, no fixed goal
                nb.updateF();
                nb.parent = current.pos;

                came_from_[nb_pos] = nb;
                open_set_.push(nb);
            }
        }
    }

    // No accepted goal found — fall back to the best rejected goal
    if (best_fallback.has_value())
        return reconstructPath(*best_fallback);

    return std::nullopt;
}

} // namespace astar

