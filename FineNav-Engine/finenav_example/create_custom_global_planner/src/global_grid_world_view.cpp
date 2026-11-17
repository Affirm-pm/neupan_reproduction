#include "global_grid_world_view.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include "finenav_core/algo_register_macros.hpp"
#include "finenav_common/frames.hpp"

namespace finenav::example {
void GlobalGridWorldView::configure(const ConfigType& cfg) {
    cfg_ = cfg;
    dirty_ = true;
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

void GlobalGridWorldView::prepare(const GlobalPlanner::Context&) {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr incoming;
    { std::lock_guard lock(mutex_); incoming = pending_; }
    if (!dirty_ && incoming == grid_) return;
    dirty_ = false;
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
    return i >= 0 && costs_[i] >= 0.0f;
}

double GlobalGridWorldView::getCost(const common::Position3D& p) const {
    const int i = cellIndex(p);
    return i >= 0 && costs_[i] >= 0.0f ? costs_[i] : 1.0;
}
}  // namespace finenav::example

FINENAV_REGISTER_WORLDVIEW(finenav::example::GlobalGridWorldView, global_grid_world_view,
                          finenav::example::GlobalPlanner, finenav::example::DummyMap)
