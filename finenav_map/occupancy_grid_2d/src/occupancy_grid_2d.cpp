// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_map_occupancy_grid_2d/occupancy_grid_2d.hpp"

#include <limits>

namespace finenav {

// ── MapConcept 接口实现 ────────────────────────────────────────────────

void OccupancyGrid2D::configure(const ConfigType& params) {
    config_ = params;
}

bool OccupancyGrid2D::isInside(const Position3D& pos) const {
    if (!grid_) return false;
    const auto& info = grid_->info;
    const double wx = pos.x() - info.origin.position.x;
    const double wy = pos.y() - info.origin.position.y;
    return wx >= 0.0 && wx < info.width  * info.resolution &&
           wy >= 0.0 && wy < info.height * info.resolution;
}

Region3D OccupancyGrid2D::getWindowBounds() const {
    if (!grid_) return Region3D(Position3D::Zero(), Position3D::Zero());
    const auto& info = grid_->info;
    constexpr double inf = std::numeric_limits<double>::infinity();
    return Region3D(
        Position3D(info.origin.position.x,
                   info.origin.position.y, -inf),
        Position3D(info.origin.position.x + info.width  * info.resolution,
                   info.origin.position.y + info.height * info.resolution, inf));
}

Position3D OccupancyGrid2D::getWindowCenter() const {
    if (!grid_) return Position3D::Zero();
    const auto& info = grid_->info;
    return Position3D(
        info.origin.position.x + info.width  * info.resolution * 0.5,
        info.origin.position.y + info.height * info.resolution * 0.5,
        0.0);
}

// ── 数据注入 ───────────────────────────────────────────────────────────

void OccupancyGrid2D::update(const nav_msgs::msg::OccupancyGrid& msg) {
    grid_ = msg;
}

bool OccupancyGrid2D::hasMap() const {
    return grid_.has_value();
}

const nav_msgs::msg::OccupancyGrid& OccupancyGrid2D::grid() const {
    return *grid_;
}

} // namespace finenav
