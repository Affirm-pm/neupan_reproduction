// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <limits>
#include <optional>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_map_occupancy_grid_2d/occupancy_grid_2d_params.hpp"

namespace finenav {
using finenav::core::Position3D;
using finenav::core::Region3D;

/**
 * @brief OccupancyGrid2D — 满足 SpatialContainer 的二维先验占用栅格地图
 *
 * 持有 nav_msgs::msg::OccupancyGrid 原始数据，对外暴露 MapConcept 查询接口。
 * 处理逻辑（二值化、代价图、膨胀）由上层 IMapView 实现。
 *
 * 不实现 shiftWindowTo —— 静态地图不可移动，MapServer 编译期跳过快门 timer。
 */
class OccupancyGrid2D {
public:
    using ConfigType = occupancy_grid_2d::Params;
    using DataType   = int8_t;  ///< occupancy value per cell: 0=free, 100=occupied, -1=unknown

    OccupancyGrid2D()                              = default;
    OccupancyGrid2D(OccupancyGrid2D&&)            = default;
    OccupancyGrid2D& operator=(OccupancyGrid2D&&) = default;

    // ── MapConcept (SpatialContainer) ────────────────────────

    void configure(const ConfigType& params);

    bool isInside(const Position3D& pos) const;

    Region3D getWindowBounds() const;

    Position3D getWindowCenter() const;

    // ── 数据注入（MapServer ObservationSource fuser 回调中调用）──

    void update(const nav_msgs::msg::OccupancyGrid& msg);

    // ── 查询接口（供 IMapView 使用）──────────────────────────

    bool hasMap() const;

    const nav_msgs::msg::OccupancyGrid& grid() const;

    const ConfigType& config() const { return config_; }

private:
    ConfigType config_{};
    std::optional<nav_msgs::msg::OccupancyGrid> grid_;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::OccupancyGrid2D, occupancy_grid_2d)
