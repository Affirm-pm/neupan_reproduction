// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <limits>
#include <optional>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/algo_configurator.hpp"
#include "finenav_map_occ_grid_2d/occupancy_grid_2d_params.hpp"

namespace finenav {
using finenav::core::Position3D;
using finenav::core::Region3D;

/**
 * @brief OccupancyGrid2D — 满足 MapConcept 的二维先验占用栅格地图数据容器
 *
 * 职责：仅持有原始 OccupancyGrid 数据，对外暴露 MapConcept 接口。
 * 处理逻辑（二值化、代价图等）由上层 IMapView 实现，不在此处进行。
 */
class OccupancyGrid2D {
public:
    using ConfigType = occupancy_grid_2d::Params;

    // DataType 代表单个格子的原始占用值
    struct CellData {
        int8_t value = -1; ///< OccupancyGrid 原始值：0=free, 100=occupied, -1=unknown
    };
    using DataType = CellData;

    OccupancyGrid2D()                              = default;
    OccupancyGrid2D(OccupancyGrid2D&&)            = default;
    OccupancyGrid2D& operator=(OccupancyGrid2D&&) = default;

    // ── MapConcept 接口 ──────────────────────────────────────

    void configure(const ConfigType& params);

    bool isInside(const Position3D& pos) const;

    Region3D getWindowBounds() const;

    Position3D getWindowCenter() const;

    void shiftWindowTo(const Position3D& /*pos*/) {} // 静态先验地图，不移动

    // ── 数据注入（MapServer fuser 回调中调用）───────────────

    void update(const nav_msgs::msg::OccupancyGrid& msg);

    // ── 原始数据访问（供 IMapView 使用）─────────────────────

    bool hasMap() const;

    const nav_msgs::msg::OccupancyGrid& grid() const;

    const ConfigType& config() const { return config_; }

private:
    ConfigType config_{};
    std::optional<nav_msgs::msg::OccupancyGrid> grid_;
};

} // namespace finenav

FINENAV_REGISTER_ALGO_CONFIG(finenav::OccupancyGrid2D, occupancy_grid_2d)
