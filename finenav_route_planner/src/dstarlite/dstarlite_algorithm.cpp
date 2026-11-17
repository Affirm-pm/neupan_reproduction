// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_route_planner/dstarlite/dstarlite_algorithm.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace dstarlite {

using astar::Vec3;
using astar::Vec3Hash;
using astar::Heuristic;
using astar::Connectivity;
using astar::calculateHeuristic;
using astar::getNeighbors;
using astar::calculateStepCost;
using finenav::Key;
using finenav::kInf;
using finenav::kKeyEpsilon;

// ===== 构造 =====

DStarLiteAlgorithm::DStarLiteAlgorithm(int rows, int cols,
                               IsWalkableFn is_walkable,
                               GetCostFn    get_cost,
                               Connectivity connectivity,
                               Heuristic    heuristic,
                               int          layers)
    : is_walkable_(std::move(is_walkable))
    , get_cost_(std::move(get_cost))
    , rows_(rows)
    , cols_(cols)
    , layers_(layers)
    , connectivity_(connectivity)
    , heuristic_type_(heuristic)
{
    // 参数校验：维度必须为正数
    if (rows <= 0 || cols <= 0 || layers <= 0) {
        throw std::invalid_argument(
            "Grid dimensions must be positive (got " +
            std::to_string(rows) + "x" + std::to_string(cols) +
            "x" + std::to_string(layers) + ")");
    }

    // 预分配状态数组并初始化为无穷大 / false
    const auto size = static_cast<size_t>(rows_ * cols_ * layers_);
    g_.assign(size, kInf);
    rhs_.assign(size, kInf);
    obstacles_.assign(size, false);
}

// ===== 连接方式 / 启发函数 =====

void DStarLiteAlgorithm::setConnectivity(Connectivity c) {
    connectivity_ = c;
}

void DStarLiteAlgorithm::setHeuristic(Heuristic h) {
    heuristic_type_ = h;
}

// ===== 栅格查询 =====

bool DStarLiteAlgorithm::isWalkable(const Vec3& p) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return is_walkable_(p);
}

double DStarLiteAlgorithm::getCost(const Vec3& p) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return get_cost_(p);
}

// ===== 障碍物修改 =====

void DStarLiteAlgorithm::setObstacle(const Vec3& p) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(p);
    if (obstacles_[idx(p)]) return;  // 已是障碍物，无需重复设置

    obstacles_[idx(p)] = true;

    if (!valid_) return;  // 尚未初始化，等 initialize() 时统一处理

    // 将该格及所有邻居标记为不一致：
    // - 被阻塞的格 g/rhs 设为无穷大，从队列中移除
    // - 所有可通行的邻居需重新计算 rhs（因为 successor 集合变了）
    g_[idx(p)]   = kInf;
    rhs_[idx(p)] = kInf;
    key_cache_.erase(p);

    for (const auto& nb : getAllNeighbors(p)) {
        if (!obstacles_[idx(nb)]) {
            updateVertex(nb);
        }
    }
}

void DStarLiteAlgorithm::removeObstacle(const Vec3& p) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(p);
    if (!obstacles_[idx(p)]) return;  // 本来就不是障碍物

    obstacles_[idx(p)] = false;

    if (!valid_) return;

    // 恢复的格可能提供新的更短路径，更新自身及所有邻居
    updateVertex(p);

    for (const auto& nb : getAllNeighbors(p)) {
        if (!obstacles_[idx(nb)]) {
            updateVertex(nb);
        }
    }
}

// ===== 地图同步 =====

void DStarLiteAlgorithm::syncMap() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // 遍历全部栅格，对比外部回调与内部记录的障碍物状态
    for (int z = 0; z < layers_; ++z) {
     for (int y = 0; y < rows_; ++y) {
      for (int x = 0; x < cols_; ++x) {
            const Vec3 p{x, y, z};
            bool real_obstacle = !is_walkable_(p);

            if (real_obstacle != obstacles_[idx(p)]) {
                if (real_obstacle) {
                    // 新增障碍物：标记并波及邻居
                    obstacles_[idx(p)] = true;
                    if (!valid_) continue;
                    g_[idx(p)]   = kInf;
                    rhs_[idx(p)] = kInf;
                    key_cache_.erase(p);
                    for (const auto& nb : getAllNeighbors(p)) {
                        if (!obstacles_[idx(nb)]) updateVertex(nb);
                    }
                } else {
                    // 障碍物被清除：更新自身及邻居
                    obstacles_[idx(p)] = false;
                    if (!valid_) continue;
                    updateVertex(p);
                    for (const auto& nb : getAllNeighbors(p)) {
                        if (!obstacles_[idx(nb)]) updateVertex(nb);
                    }
                    }
                }
            }
        }
    }
}

void DStarLiteAlgorithm::syncMapAround(const Vec3& around, int radius_cells) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    // 计算搜索范围（裁剪到地图边界内）
    int x0 = std::max(0, around.x - radius_cells);
    int x1 = std::min(cols_ - 1, around.x + radius_cells);
    int y0 = std::max(0, around.y - radius_cells);
    int y1 = std::min(rows_ - 1, around.y + radius_cells);
    int z0 = std::max(0, around.z - radius_cells);
    int z1 = std::min(layers_ - 1, around.z + radius_cells);

    // 仅在局部范围内扫描，逻辑同 syncMap()
    for (int z = z0; z <= z1; ++z) {
     for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
            const Vec3 p{x, y, z};
            bool real_obstacle = !is_walkable_(p);

            if (real_obstacle != obstacles_[idx(p)]) {
                if (real_obstacle) {
                    obstacles_[idx(p)] = true;
                    if (!valid_) continue;
                    g_[idx(p)]   = kInf;
                    rhs_[idx(p)] = kInf;
                    key_cache_.erase(p);
                    for (const auto& nb : getAllNeighbors(p)) {
                        if (!obstacles_[idx(nb)]) updateVertex(nb);
                    }
                } else {
                    obstacles_[idx(p)] = false;
                    if (!valid_) continue;
                    updateVertex(p);
                    for (const auto& nb : getAllNeighbors(p)) {
                        if (!obstacles_[idx(nb)]) updateVertex(nb);
                    }
                }
            }
          }
         }
        }
}

// ===== 算法生命周期 =====

void DStarLiteAlgorithm::initialize(const Vec3& start, const Vec3& goal) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(start);
    checkBounds(goal);

    // 步骤 1：从外部回调同步全部障碍物信息
    for (int z = 0; z < layers_; ++z) {
     for (int y = 0; y < rows_; ++y) {
      for (int x = 0; x < cols_; ++x) {
            const Vec3 p{x, y, z};
            obstacles_[idx(p)] = !is_walkable_(p);
      }
     }
    }

    // 步骤 2：清空全部 g/rhs 值为无穷大
    std::fill(g_.begin(), g_.end(), kInf);
    std::fill(rhs_.begin(), rhs_.end(), kInf);

    // 步骤 3：清空优先队列和 key 缓存
    U_ = decltype(U_){};
    key_cache_.clear();

    start_ = start;

    // 步骤 4：目标容忍 —— 若目标被阻塞，在容忍范围内搜索最近的可通行格
    Vec3 effective_goal = goal;
    if (!is_walkable_(goal) && goal_tolerance_ > 0) {
        int best_dist = goal_tolerance_ + 1;
        for (int dz = -goal_tolerance_; dz <= goal_tolerance_; ++dz) {
         for (int dy = -goal_tolerance_; dy <= goal_tolerance_; ++dy) {
          for (int dx = -goal_tolerance_; dx <= goal_tolerance_; ++dx) {
                int nz = goal.z + dz;
                int ny = goal.y + dy;
                int nx = goal.x + dx;
                if (nz < 0 || nz >= layers_ ||
                    ny < 0 || ny >= rows_ ||
                    nx < 0 || nx >= cols_) continue;

                int dist = std::abs(dz) + std::abs(dy) + std::abs(dx);
                const Vec3 candidate{nx, ny, nz};
                if (dist <= goal_tolerance_ && dist < best_dist &&
                    is_walkable_(candidate)) {
                    best_dist = dist;
                    effective_goal = candidate;
                }
          }
         }
        }
        // 容忍范围内未找到可通行格，发出警告
        if (best_dist > goal_tolerance_) {
            std::cerr << "[DStarLiteAlgorithm] WARN: goal (" << goal.x
                      << "," << goal.y << ") blocked and no walkable cell"
                      << " within tolerance " << goal_tolerance_ << ".\n";
        }
    }

    // 步骤 5：设置目标 rhs=0 并插入优先队列（搜索从目标反向扩展到起点）
    goal_ = effective_goal;
    km_   = 0.0;

    rhs_[idx(goal_)] = 0.0;
    insertOrUpdate(goal_);

    valid_ = true;
}

void DStarLiteAlgorithm::computeShortestPath() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    int expansions = 0;

    const auto t_start = (max_planning_time_ > 0.0)
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};

    while (!U_.empty()) {
        // 扩展次数限制检查
        if (max_expansions_ > 0 && ++expansions > max_expansions_) {
            std::cerr << "[DStarLiteAlgorithm] ERROR: computeShortestPath "
                      << "exceeded " << max_expansions_
                      << " expansions — aborting.\n";
            break;
        }

        // 时间限制检查
        if (max_planning_time_ > 0.0) {
            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start);
            if (elapsed.count() > max_planning_time_) {
                std::cerr << "[DStarLiteAlgorithm] WARN: computeShortestPath "
                          << "exceeded " << max_planning_time_
                          << " s time limit — aborting.\n";
                break;
            }
        }

        const auto [top_key, top_node] = U_.top();

        // 终止条件：起点达到一致（g == rhs）且其 Key 不大于队首 Key，
        // 即起点的路径已经是最优的
        const Key start_key = calculateKey(start_);
        if (!(top_key < start_key) &&
            !(rhs_[idx(start_)] > g_[idx(start_)])) {
            break;
        }

        U_.pop();

        // 惰性删除：跳过优先队列中的过期条目
        auto it = key_cache_.find(top_node);
        if (it == key_cache_.end() || !(top_key == it->second)) {
            continue;
        }
        key_cache_.erase(top_node);

        const double g_u   = g_[idx(top_node)];
        const double rhs_u = rhs_[idx(top_node)];

        if (g_u > rhs_u) {
            // 过一致(over-consistent)：存在更短的路径，传播改进
            g_[idx(top_node)] = rhs_u;
            for (const auto& s : getPredecessors(top_node)) {
                updateVertex(s);
            }
        } else {
            // 欠一致(under-consistent)：原有路径变差，重置并传播
            g_[idx(top_node)] = kInf;
            updateVertex(top_node);
            for (const auto& s : getPredecessors(top_node)) {
                updateVertex(s);
            }
        }
    }
}

void DStarLiteAlgorithm::replan(const Vec3& new_start) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(new_start);

    // 根据机器人移动距离更新 km 偏移量，确保启发式一致性
    km_ += calculateHeuristic(heuristic_type_, start_, new_start);
    start_ = new_start;

    computeShortestPath();
}

bool DStarLiteAlgorithm::step() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (start_ == goal_) return true;  // 已到达目标

    const auto neighbors = getSuccessors(start_);
    if (neighbors.empty()) return false;  // 无路可走

    // 贪心选择：从所有后继中选择 moveCost + g 最小的邻居
    Vec3   best      = neighbors[0];
    double best_cost = kInf;

    for (const auto& nb : neighbors) {
        double cost_val = moveCost(start_, nb) + g_[idx(nb)];
        if (cost_val < best_cost) {
            best_cost = cost_val;
            best      = nb;
        }
    }

    if (best_cost >= kInf * 0.5) return false;  // 无可达邻居

    start_ = best;   // 注意：此处不更新 km（符合论文规范）
    return true;
}

// ===== 路径提取 =====

std::vector<Vec3> DStarLiteAlgorithm::extractPath() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<Vec3> path;
    Vec3 current = start_;
    path.push_back(current);

    int steps = 0;

    // 沿 g 值梯度下降方向提取路径：每步选择 moveCost + g 最小的后继
    while (current != goal_ && (max_path_steps_ < 0 || steps < max_path_steps_)) {
        ++steps;

        const auto neighbors = getSuccessors(current);
        if (neighbors.empty()) break;

        Vec3   best     = neighbors[0];
        double best_val = kInf;

        for (const auto& nb : neighbors) {
            double val = moveCost(current, nb) + g_[idx(nb)];
            if (val < best_val) {
                best_val = val;
                best     = nb;
            }
        }

        if (best_val >= kInf * 0.5) break;  // 后继全部不可达

        current = best;
        path.push_back(current);
    }

    // 若路径末端未到达目标，视为不可达
    if (path.back() != goal_) return {};
    return path;
}

// ===== 访问器 =====

double DStarLiteAlgorithm::g(const Vec3& p) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(p);
    return g_[idx(p)];
}

double DStarLiteAlgorithm::rhs(const Vec3& p) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    checkBounds(p);
    return rhs_[idx(p)];
}

// ===== 调试 =====

void DStarLiteAlgorithm::printGrid(const std::vector<Vec3>& path) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::unordered_map<Vec3, bool, Vec3Hash> on_path;
    for (const auto& p : path) on_path[p] = true;

    // 多层地图仅展示 z=0 层
    int z_show = 0;
    if (layers_ > 1) {
        std::cout << "[DStarLiteAlgorithm] " << layers_ << " layers; showing z=0.\n";
    }

    // S = 起点, G = 目标, # = 障碍物, * = 路径上的格, . = 空地
    for (int y = 0; y < rows_; ++y) {
        for (int x = 0; x < cols_; ++x) {
            Vec3 p{x, y, z_show};
            if (p == start_)             std::cout << "S ";
            else if (p == goal_)         std::cout << "G ";
            else if (obstacles_[idx(p)]) std::cout << "# ";
            else if (on_path.contains(p)) std::cout << "* ";
            else                          std::cout << ". ";
        }
        std::cout << '\n';
    }
    std::cout << '\n';
}

// ===== D* Lite 核心函数 =====

Key DStarLiteAlgorithm::calculateKey(const Vec3& s) const {
    // Key = [min(g, rhs) + h(start, s) * greedy_weight + km,  min(g, rhs)]
    // 第一分量 k1 决定优先队列中的排序
    // 第二分量 k2 用于打破 k1 相等时的平局
    const double ming  = std::min(g_[idx(s)], rhs_[idx(s)]);
    const double h_val = calculateHeuristic(heuristic_type_, start_, s) * greedy_weight_;
    return {ming + h_val + km_, ming};
}

void DStarLiteAlgorithm::updateVertex(const Vec3& u) {
    // 被阻塞的节点不可达，从队列中移除
    if (obstacles_[idx(u)]) {
        g_[idx(u)]   = kInf;
        rhs_[idx(u)] = kInf;
        key_cache_.erase(u);
        return;
    }

    // 非目标节点：rhs(u) = min_{s∈Succ(u)}( moveCost(u, s) + g(s) )
    if (u != goal_) {
        double best = kInf;
        for (const auto& sp : getSuccessors(u)) {
            double candidate = moveCost(u, sp) + g_[idx(sp)];
            if (candidate < best) best = candidate;
        }
        rhs_[idx(u)] = best;
    }

    // 若节点变为一致(g == rhs)，从队列中移除
    // 若节点不一致(g != rhs)，插入或更新队列条目
    auto it = key_cache_.find(u);
    if (it != key_cache_.end()) key_cache_.erase(it);

    if (g_[idx(u)] != rhs_[idx(u)]) {
        insertOrUpdate(u);
    }
}

void DStarLiteAlgorithm::insertOrUpdate(const Vec3& s) {
    // 计算当前 Key 并插入优先队列，同时在 key_cache_ 中记录
    // key_cache_ 用于惰性删除：当弹出节点的 Key 与缓存不一致时跳过
    Key k = calculateKey(s);
    key_cache_[s] = k;
    U_.emplace(k, s);
}

// ===== 邻域与代价（委托给 grid_search_utils.hpp 中的共享函数）=====

std::vector<Vec3> DStarLiteAlgorithm::getSuccessors(const Vec3& u) const {
    std::vector<Vec3> result;
    const auto neighbors = getNeighbors(u, connectivity_);
    result.reserve(neighbors.size());

    // 后继 = 在栅格范围内且可通行的邻居
    for (const auto& nb : neighbors) {
        if (nb.x >= 0 && nb.x < cols_ &&
            nb.y >= 0 && nb.y < rows_ &&
            nb.z >= 0 && nb.z < layers_ &&
            !obstacles_[idx(nb)]) {
            result.push_back(nb);
        }
    }
    return result;
}

std::vector<Vec3> DStarLiteAlgorithm::getPredecessors(const Vec3& u) const {
    // 无向栅格图中前驱集 = 后继集
    return getSuccessors(u);
}

std::vector<Vec3> DStarLiteAlgorithm::getAllNeighbors(const Vec3& u) const {
    std::vector<Vec3> result;
    const auto neighbors = getNeighbors(u, connectivity_);
    result.reserve(neighbors.size());

    // 所有邻居（含障碍物），用于障碍物修改时的波及更新
    for (const auto& nb : neighbors) {
        if (nb.x >= 0 && nb.x < cols_ &&
            nb.y >= 0 && nb.y < rows_ &&
            nb.z >= 0 && nb.z < layers_) {
            result.push_back(nb);
        }
    }
    return result;
}

double DStarLiteAlgorithm::moveCost(const Vec3& a, const Vec3& b) const {
    // 边代价 = 步长代价 × (movement_weight_ + terrain_weight_ × 地形代价)
    // 例如相邻格步长=1.0，对角格步长=√2，3-D对角格步长=√3
    const double base = calculateStepCost(a, b);
    return base * (movement_weight_ + terrain_weight_ * get_cost_(b));
}

// ===== 辅助函数 =====

void DStarLiteAlgorithm::checkBounds(const Vec3& p) const {
    if (p.x < 0 || p.x >= cols_ ||
        p.y < 0 || p.y >= rows_ ||
        p.z < 0 || p.z >= layers_) {
        throw std::out_of_range(
            "Vec3 (" + std::to_string(p.x) + "," + std::to_string(p.y) +
            "," + std::to_string(p.z) + ") out of bounds [0," +
            std::to_string(cols_ - 1) + "]x[0," + std::to_string(rows_ - 1) +
            "]x[0," + std::to_string(layers_ - 1) + "]");
    }
}

}  // namespace dstarlite
