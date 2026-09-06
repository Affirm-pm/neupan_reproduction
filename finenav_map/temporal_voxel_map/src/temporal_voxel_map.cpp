// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_map_temporal_voxel_map/temporal_voxel_map.hpp"

#include <cmath>

#include "finenav_core/common/algo_register_macros.hpp"

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

    grid_.configure(GridMapConfig{
        .length     = Length(cfg.map_length, cfg.map_length, cfg.map_length),
        .resolution = cfg.resolution,
    });

    RCLCPP_INFO(logger_,
        "TemporalVoxelMap configured: "
        "res=%.2f m, length=%.1f m, FOV azimuth=%.0f°→%.0f° (XY plane, +X=0°), "
        "range=%.1f-%.1f m, decay=%.1f/%.1f s",
        cfg.resolution, cfg.map_length,
        cfg.min_angle * 180.0 / M_PI, cfg.max_angle * 180.0 / M_PI,
        cfg.min_range, cfg.max_range,
        cfg.decay_time_sec, cfg.decay_time_fov_sec);
}

// ==============================================================================
// FOV check (azimuth sector in XY plane, +X = forward)
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

    // Rotate to body frame, compute azimuth in XY plane.
    // 0 = +X (forward), +π/2 = +Y (left), -π/2 = -Y (right).
    const Eigen::Vector3d v_body = T_map_body.linear().transpose() * v_world;
    const double azi = std::atan2(v_body.y(), v_body.x());

    // Normalise and compare to [min, max] with wraparound support.
    double a   = normalizeAngle(azi);
    double s   = normalizeAngle(min_angle_rad_);
    double e   = normalizeAngle(max_angle_rad_);
    double mid = normalizeAngle(s + normalizeAngle(e - s) * 0.5);

    double dev   = normalizeAngle(a - mid);
    double width = normalizeAngle(e - s);
    return std::abs(dev) <= std::abs(width) * 0.5 + 1e-9;
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

    // FOV visualization
    if (viz_) {
        viz_->markers().setBaseFrame("map");
        viz_->markers().addCone(T_map_body, max_angle_rad_, max_range_,
                                finenav::viz::getColor(finenav::viz::ColorName::TRANSLUCENT));
        viz_->publishAll();
    }
}

}  // namespace finenav

// One-line registration: parameter wiring + map self-registration.
// "temporal_voxel_map" must match the top-level key in temporal_voxel_map.yaml.
FINENAV_REGISTER_MAP(finenav::TemporalVoxelMap, temporal_voxel_map)
