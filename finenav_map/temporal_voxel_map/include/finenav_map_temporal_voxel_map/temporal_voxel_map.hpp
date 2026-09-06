// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <memory>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "finenav_core/map/map_concept.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"
#include "finenav_map_grid_map/grid_map.hpp"
#include "finenav_map_temporal_voxel_map/temporal_voxel_map_params.hpp"
#include "finenav_map_temporal_voxel_map/voxel.hpp"

namespace finenav {

using finenav::core::Position3D;
using finenav::core::Region3D;

/**
 * @brief TemporalVoxelMap — 3D voxel grid with time-based dynamic obstacle filtering.
 *
 * Wraps GridMap<Voxel> internally and adds:
 *   - Point-cloud insertion with azimuth FOV filtering (XY plane, +X forward)
 *   - Time-based pruning of expired dynamic voxels
 *
 * Satisfies SpatialContainer and ShiftableMap — usable with MapServer.
 * All methods are called from the MapServer update thread (write lock held),
 * so no additional locking is needed.
 */
class TemporalVoxelMap {
public:
    using ConfigType = temporal_voxel_map::Params;
    using DataType   = Voxel;

    TemporalVoxelMap() = default;

    /// Receive FineNavVisualizer injected by AlgoConfigurator.
    void setVisualizer(finenav::viz::FineNavVisualizer& viz) { viz_ = &viz; }
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

    // ── MapConcept ─────────────────────────────────────────────────

    void configure(const ConfigType& cfg);

    bool isInside(const finenav::core::Position3D& pos) const {
        return grid_.isInside(Position{pos.x, pos.y, pos.z});
    }

    finenav::core::Region3D getWindowBounds() const {
        const auto b = grid_.getWindowBounds();  // internal Eigen::AlignedBox3d
        return finenav::core::Region3D{
            finenav::core::Position3D{b.min().x(), b.min().y(), b.min().z()},
            finenav::core::Position3D{b.max().x(), b.max().y(), b.max().z()}};
    }

    finenav::core::Position3D getWindowCenter() const {
        const Position c = grid_.getWindowCenter();  // internal Eigen::Vector3d
        return finenav::core::Position3D{c.x(), c.y(), c.z()};
    }

    void shiftWindowTo(const finenav::core::Position3D& pos) {
        grid_.shiftWindowTo(Position{pos.x, pos.y, pos.z});
    }

    void selfManage(finenav::core::MapRuntime&) {}

    // ── Sensor data injection ─────────────────────────────────────

    /**
     * @brief Transform and insert a point cloud into the voxel grid.
     *
     * Points outside the azimuth FOV (XY plane, +X forward) or outside the map bounds are skipped.
     * Each inserted voxel's timestamp is set to steady_clock::now() and its
     * position is added to the dynamic-cell set for later pruning.
     *
     * @param cloud       Point cloud in body frame.
     * @param T_map_body  Body-to-map transform.
     */
    void insertPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                          const Eigen::Isometry3d& T_map_body);

    // ── Temporal decay ────────────────────────────────────────────

    /**
     * @brief Remove voxels whose timestamp exceeds the configured lifetime.
     *
     * Called from MapServer's preUpdateHook.  Voxels inside the FOV use
     * decay_time_fov_sec_; those outside use decay_time_sec_.
     *
     * @param T_map_body  Current body-to-map transform (for FOV check).
     */
    void pruneExpiredCells(const Eigen::Isometry3d& T_map_body);

private:
    /// Azimuth FOV in XY plane: +X=0° forward, sector [min_angle, max_angle].
    bool isInFrustum(const Position& p_world,
                     const Eigen::Isometry3d& T_map_body) const;

    static double normalizeAngle(double angle);

    // ── Internal state ────────────────────────────────────────────

    GridMap<Voxel> grid_;
    DynamicPosSet   dynamic_cells_;

    // ── Parameters (set by configure) ─────────────────────────────

    double min_range_          = 0.0;
    double max_range_          = 50.0;
    double min_angle_rad_      = 0.0;
    double max_angle_rad_      = 1.047;   // π/3
    double decay_time_sec_     = 10.0;
    double decay_time_fov_sec_ = 1.0;

    rclcpp::Logger                logger_{rclcpp::get_logger("TemporalVoxelMap")};
    finenav::viz::FineNavVisualizer* viz_ = nullptr;
};

}  // namespace finenav
