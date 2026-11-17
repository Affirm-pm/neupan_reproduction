#include "global_grid_world_view.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include "finenav_core/algo_register_macros.hpp"
#include "finenav_common/frames.hpp"

namespace finenav::example {
void GlobalGridWorldView::configure(const ConfigType& cfg) {
    live_cells_.clear();
    cfg_ = cfg;
    dirty_ = true;
    if (cfg_.use_live_obstacles && !observation_params_) {
        observation_params_ = std::make_unique<neupan_world_view::ParamListener>(
            node_, "global_grid_world_view.obstacles");
        observations_.setLogger(node_->get_logger());
    }
    // Subscribe at layer startup, before a navigation goal starts prepare().
    // Otherwise the first plan faults before any observation can arrive.
    if (cfg_.use_live_obstacles) observations_.configure(observation_params_->get_params());
    if (!subscription_) {
        // Humble does not support transient-local durability with intra-process
        // delivery; FineNav enables intra-process by default on layer nodes.
        rclcpp::SubscriptionOptions options;
        options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
        subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            cfg.map_topic, rclcpp::QoS(1).transient_local().reliable(),
            [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
                std::lock_guard lock(mutex_);
                pending_ = std::move(msg);
            }, options);
    }
}

void GlobalGridWorldView::prepare(const GlobalPlanner::Context& ctx) {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr incoming;
    { std::lock_guard lock(mutex_); incoming = pending_; }
    // A new lidar observation must rebuild the overlay even with a cached map.
    if (!dirty_ && incoming == grid_ && !cfg_.use_live_obstacles) return;
    dirty_ = false;
    if (!incoming || !grid_ ||
        (incoming->info.width != grid_->info.width || incoming->info.height != grid_->info.height ||
         incoming->info.resolution != grid_->info.resolution || incoming->info.origin != grid_->info.origin ||
         incoming->header.frame_id != grid_->header.frame_id)) live_cells_.clear();
    grid_ = incoming;
    valid_ = false;
    costs_.clear();
    if (!grid_) return;
    const auto& m = grid_->info;
    const auto& q = m.origin.orientation;
    // This bridge deliberately requires an axis-aligned world-frame grid.
    // Reject incompatible frames instead of silently planning in the wrong coordinates.
    if (grid_->header.frame_id != finenav::common::frames().world_fixed ||
        !std::isfinite(m.resolution) || m.resolution < 0.01f || m.resolution > 2.0 ||
        m.width == 0 || m.height == 0 || m.width > 10000 || m.height > 10000 ||
        static_cast<uint64_t>(m.width) * m.height > 10000000 ||
        grid_->data.size() != static_cast<size_t>(m.width) * m.height ||
        !std::isfinite(m.origin.position.x) || !std::isfinite(m.origin.position.y) ||
        !std::isfinite(m.origin.position.z) || std::abs(m.origin.position.z) > 1e-6 ||
        std::abs(m.origin.position.x / m.resolution) > 1e8 ||
        std::abs(m.origin.position.y / m.resolution) > 1e8 ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
        std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 || std::abs(q.z) > 1e-6 ||
        std::abs(std::abs(q.w) - 1.0) > 1e-6 ||
        !std::isfinite(cfg_.inflation_radius) || cfg_.inflation_radius < 0.0) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid global grid: require an axis-aligned world-frame map, valid dimensions/resolution and finite origin.");
        return;
    }

    // A blocked padding border makes outside-map space an obstacle too.
    cv::Mat free(m.height + 2, m.width + 2, CV_8UC1, cv::Scalar(0));
    for (size_t y = 0; y < m.height; ++y)
        for (size_t x = 0; x < m.width; ++x) {
            const int cell = grid_->data[y * m.width + x];
            free.at<uint8_t>(y + 1, x + 1) = (cell >= 0 && cell < cfg_.occupied_threshold) ? 255 : 0;
        }
    if (cfg_.use_live_obstacles) {
        observations_.configure(observation_params_->get_params());
        const core::InputTrajectoryT<NeuPanPlanner> unused_reference;
        observations_.prepare({ctx.robot_state, unused_reference});
        if (!observations_.observationValid()) {
            RCLCPP_WARN(node_->get_logger(),
                "Global replan requires a fresh, transformable obstacle observation.");
            return;
        }
        // Briefly retain occupied cells through occlusion or missing returns:
        // a single empty scan must not restore a path through the same box.
        // Never mutate /global_map or use optimizer decimation here.
        const auto now = std::chrono::steady_clock::now();
        for (const auto& p : observations_.collisionPoints()) {
            const double x = std::floor((p.x - m.origin.position.x) / m.resolution);
            const double y = std::floor((p.y - m.origin.position.y) / m.resolution);
            if (x >= 0 && y >= 0 && x < m.width && y < m.height) {
                const size_t cell = static_cast<size_t>(y) * m.width + static_cast<size_t>(x);
                auto it = live_cells_.find(cell);
                if (it == live_cells_.end() ||
                    std::chrono::duration<double>(now - it->second.stamp).count() > cfg_.obstacle_persistence)
                    live_cells_[cell] = {p.x, p.x, p.y, p.y, now};
                else {
                    auto& c = it->second;
                    c.min_x = std::min(c.min_x, p.x); c.max_x = std::max(c.max_x, p.x);
                    c.min_y = std::min(c.min_y, p.y); c.max_y = std::max(c.max_y, p.y);
                    c.stamp = now;
                }
            }
        }
        for (auto it = live_cells_.begin(); it != live_cells_.end();) {
            if (std::chrono::duration<double>(now - it->second.stamp).count() > cfg_.obstacle_persistence)
                it = live_cells_.erase(it);
            else {
                free.at<uint8_t>(it->first / m.width + 1, it->first % m.width + 1) = 0;
                ++it;
            }
        }
    }
    cv::Mat distance;
    cv::distanceTransform(free, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    costs_.resize(grid_->data.size());
    // Two half-diagonals account for occupied cell area and arbitrary query
    // locations (including the upstream grid's differently anchored centres).
    const double clearance = cfg_.inflation_radius + std::sqrt(2.0) * m.resolution;
    for (size_t y = 0; y < m.height; ++y)
        for (size_t x = 0; x < m.width; ++x) {
            const double margin = distance.at<float>(y + 1, x + 1) * m.resolution - clearance;
            // Prefer routes away from the inflation boundary, leaving room for
            // the differential drive to round corners while tracking the route.
            costs_[y * m.width + x] = margin <= 0.0 ? -1.0 :
                std::max(0.0, 1.0 - margin / cfg_.clearance_cost_distance);
        }
    start_x_ = ctx.robot_state.pose.position.x;
    start_y_ = ctx.robot_state.pose.position.y;
    valid_ = true;
}

int GlobalGridWorldView::cellIndex(const common::Position3D& p) const {
    if (!valid_ || !std::isfinite(p.x) || !std::isfinite(p.y)) return -1;
    const auto& m = grid_->info;
    const double x = std::floor((p.x - m.origin.position.x) / m.resolution);
    const double y = std::floor((p.y - m.origin.position.y) / m.resolution);
    if (x < 0 || y < 0 || x >= m.width || y >= m.height) return -1;
    return static_cast<int>(y) * static_cast<int>(m.width) + static_cast<int>(x);
}

bool GlobalGridWorldView::isWalkable(const common::Position3D& p) const {
    const int i = cellIndex(p);
    if (i < 0) return false;
    if (costs_[i] >= 0.0f) return true;
    // A safe robot can drift into extra raster padding while rounding a
    // corner or observing a new object. Check exact query positions near the
    // start, rather than rejecting the episode or erasing occupied cells.
    if (!cfg_.use_live_obstacles || !std::isfinite(start_x_) || !std::isfinite(start_y_) ||
        std::hypot(p.x-start_x_, p.y-start_y_) > .8) return false;
    const auto& m = grid_->info;
    // Preserve inflation_radius, plus half the maximum connector sample gap.
    const double required = cfg_.inflation_radius + m.resolution*.125;
    if (std::min({p.x-m.origin.position.x, p.y-m.origin.position.y,
                  m.origin.position.x+m.width*m.resolution-p.x,
                  m.origin.position.y+m.height*m.resolution-p.y}) < required) return false;
    const int x0 = std::max(0, static_cast<int>(std::floor((p.x-required-m.origin.position.x)/m.resolution)));
    const int y0 = std::max(0, static_cast<int>(std::floor((p.y-required-m.origin.position.y)/m.resolution)));
    const int x1 = std::min(static_cast<int>(m.width)-1, static_cast<int>(std::floor((p.x+required-m.origin.position.x)/m.resolution)));
    const int y1 = std::min(static_cast<int>(m.height)-1, static_cast<int>(std::floor((p.y+required-m.origin.position.y)/m.resolution)));
    for (int y=y0; y<=y1; ++y) for (int x=x0; x<=x1; ++x) {
        const int cell = grid_->data[y*m.width+x];
        if (cell >= 0 && cell < cfg_.occupied_threshold) continue;
        const double left=m.origin.position.x+x*m.resolution, bottom=m.origin.position.y+y*m.resolution;
        if (std::hypot(std::max({left-p.x,p.x-left-m.resolution,0.0}),
                       std::max({bottom-p.y,p.y-bottom-m.resolution,0.0})) < required) return false;
    }
    return std::all_of(live_cells_.begin(), live_cells_.end(), [&](const auto& entry) {
        const auto& c = entry.second;
        return std::hypot(std::max({c.min_x-p.x,p.x-c.max_x,0.0}),
                          std::max({c.min_y-p.y,p.y-c.max_y,0.0})) >= required;
    });
}

double GlobalGridWorldView::getCost(const common::Position3D& p) const {
    const int i = cellIndex(p);
    return i >= 0 && costs_[i] >= 0.0f ? costs_[i] : 1.0;
}
}  // namespace finenav::example

FINENAV_REGISTER_WORLDVIEW(finenav::example::GlobalGridWorldView, global_grid_world_view,
                          finenav::example::GlobalPlanner, finenav::example::DummyMap)
