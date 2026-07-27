// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_map_temporal_voxel_map/temporal_voxel_map.hpp"

#include <cmath>

namespace finenav {

// ==============================================================================
// Utility
// ==============================================================================

double TemporalVoxelMap::normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ==============================================================================
// MapConcept
// ==============================================================================

void TemporalVoxelMap::configure(const ConfigType& cfg) {
    min_range_          = cfg.min_range;
    max_range_          = cfg.max_range;
    min_angle_rad_      = cfg.min_angle;
    max_angle_rad_      = cfg.max_angle;
    decay_time_sec_     = cfg.decay_time_sec;
    decay_time_fov_sec_ = cfg.decay_time_fov_sec;

    // Configure the underlying GridMap
    grid_.configure(GridMapConfig{
        .length     = Length(cfg.map_length, cfg.map_length, cfg.map_length),
        .resolution = cfg.resolution,
    });
}

// ==============================================================================
// FOV check (single cone, axis = body +Z)
// ==============================================================================

bool TemporalVoxelMap::isInFrustum(const Position& p_world,
                                    const Eigen::Isometry3d& T_map_body) const
{
    const Eigen::Vector3d origin = T_map_body.translation();
    const Eigen::Vector3d v_world = p_world - origin;

    const double r = v_world.norm();
    if (r <= 0.0 || r < min_range_ || r > max_range_) {
        return false;
    }

    // Transform to body frame; +Z is sensor axis
    const Eigen::Vector3d v_sensor = T_map_body.linear().transpose() * v_world;
    const double norm = v_sensor.norm();
    if (norm <= 0.0) {
        return false;
    }

    const double cos_theta = std::clamp(v_sensor.z() / norm, -1.0, 1.0);
    const double theta = std::acos(cos_theta);
    return theta >= min_angle_rad_ && theta <= max_angle_rad_;
}

// ==============================================================================
// InsertPointCloud
// ==============================================================================

void TemporalVoxelMap::insertPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                         const Eigen::Isometry3d& T_map_body)
{
    const auto now = std::chrono::steady_clock::now();

    for (const auto& pt : cloud) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        const Eigen::Vector3d p_world = T_map_body * Eigen::Vector3d(pt.x, pt.y, pt.z);
        const Position hit_pos(p_world.x(), p_world.y(), p_world.z());

        if (!grid_.isInside(hit_pos)) {
            continue;
        }

        if (!isInFrustum(hit_pos, T_map_body)) {
            continue;
        }

        const Index idx = grid_.getIndex(hit_pos);
        Voxel& cell = grid_.at(idx);
        if (cell.is_static) {
            continue;
        }

        cell.z_height  = static_cast<float>(p_world.z());
        cell.timestamp = now;
        dynamic_cells_.insert(grid_.getPosition(idx));
    }
}

// ==============================================================================
// PruneExpiredCells
// ==============================================================================

void TemporalVoxelMap::pruneExpiredCells(const Eigen::Isometry3d& T_map_body)
{
    const auto now = std::chrono::steady_clock::now();

    for (auto it = dynamic_cells_.begin(); it != dynamic_cells_.end();) {
        const Position& pos = *it;

        if (!grid_.isInside(pos)) {
            it = dynamic_cells_.erase(it);
            continue;
        }

        Voxel& cell = grid_.atPosition(pos);
        if (cell.timestamp.time_since_epoch().count() == 0) {
            it = dynamic_cells_.erase(it);
            continue;
        }

        const double age_sec =
            std::chrono::duration<double>(now - cell.timestamp).count();

        if (age_sec > decay_time_sec_) {
            cell = Voxel{};
            it = dynamic_cells_.erase(it);
            continue;
        }

        const bool in_fov = isInFrustum(pos, T_map_body);
        const double active_decay = in_fov ? decay_time_fov_sec_ : decay_time_sec_;

        if (age_sec > active_decay) {
            cell = Voxel{};
            it = dynamic_cells_.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace finenav
