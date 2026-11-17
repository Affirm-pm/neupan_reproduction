// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <limits>
#include <cmath>

namespace astar {

// Test grid with integer (x, y) cell queries and traversal costs.
class GridMap {
public:
    GridMap(int width, int height, double default_cost = 0.0)
        : width_(width), height_(height),
          costs_(width * height, default_cost)
    {}

    // setters
    void setObstacle(int x, int y, bool obstacle) {
        if (isValid(x, y))
            costs_[idx(x, y)] = obstacle ? INF_COST : 0.0;
    }

    void setCost(int x, int y, double cost) {
        if (isValid(x, y))
            costs_[idx(x, y)] = cost;
    }

    // cell-space queries (used by a wrapping IWorldView)
    bool isWalkableCell(int x, int y) const {
        return isValid(x, y) && costs_[idx(x, y)] < INF_COST;
    }

    double getCostCell(int x, int y) const {
        return isValid(x, y) ? costs_[idx(x, y)] : INF_COST;
    }

    // metadata
    int getWidth()  const { return width_;  }
    int getHeight() const { return height_; }

private:
    bool isValid(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }
    int idx(int x, int y) const { return y * width_ + x; }

    int    width_, height_;
    std::vector<double> costs_;
    static constexpr double INF_COST = 1e9;
};

} // namespace astar

