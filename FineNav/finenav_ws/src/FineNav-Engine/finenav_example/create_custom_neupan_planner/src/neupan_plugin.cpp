// SPDX-License-Identifier: GPL-3.0-or-later
#include "neupan_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {
namespace {
double yaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                      1.0 - 2.0 * (q.y*q.y + q.z*q.z));
}
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
bool validPose(const geometry_msgs::msg::Pose& p) {
    const auto& q = p.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
           std::isfinite(p.position.z) && std::isfinite(norm) && std::abs(norm-1.0)<0.01;
}
bool recent(const builtin_interfaces::msg::Time& stamp, const rclcpp::Time& now, double timeout) {
    const double age = (now-rclcpp::Time(stamp, now.get_clock_type())).seconds();
    return age >= -0.05 && age <= timeout;
}
}

void NeuPANCloudMap::selfManage(core::MapRuntime& runtime) {
    core::ObservationBufferPolicy policy;
    policy.max_buffer_size = 8;
    policy.observation_keep_time_sec = 0.5;
    runtime.addObservationSource<DataType>("neupan_cloud", topic_, rclcpp::SensorDataQoS(), policy,
        [this](const DataType& cloud, const rclcpp::Time&, const core::RobotState&) {
            cloud_ = std::make_unique<DataType>(cloud);
            received_ = Steady::now();
        });
}

void NeuPANWorldView::configure(const ConfigType& cfg) {
    if (!(cfg.data_timeout > 0) || !(cfg.voxel_size > 0) || cfg.voxel_size > 0.04 ||
        !(cfg.max_range >= 1.0) || !std::isfinite(cfg.max_range) ||
        !std::isfinite(cfg.data_timeout) || !std::isfinite(cfg.min_height) ||
        !std::isfinite(cfg.max_height) || !(cfg.max_height > cfg.min_height))
        throw std::invalid_argument("Invalid NeuPAN point cloud filter/timeouts");
    cfg_ = cfg;
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        cfg.pose_topic, rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
            std::lock_guard lock(pose_mutex_);
            pose_ = msg;
            pose_received_ = Steady::now();
        });
}

void NeuPANWorldView::prepare(const NeuPANPlanner::Context& ctx) {
    ready_ = false;
    points_.resize(2, 0);
    auto fail = [this](const char* reason) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                            "NeuPAN holds zero velocity: %s", reason);
    };
    const auto now = node_->now();
    const auto steady = Steady::now();
    const auto* cloud = map_.cloud();
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose;
    Steady::time_point received;
    { std::lock_guard lock(pose_mutex_); pose = pose_; received = pose_received_; }
    if (!pose || !validPose(pose->pose.pose) ||
        pose->header.frame_id != common::frames().world_fixed ||
        !recent(pose->header.stamp, now, cfg_.data_timeout) ||
        std::chrono::duration<double>(steady-received).count() > cfg_.data_timeout) {
        fail("missing, stale or invalid localization_pose"); return;
    }
    if (!cloud || cloud->header.frame_id != common::frames().world_fixed ||
        !recent(cloud->header.stamp, now, cfg_.data_timeout) ||
        std::chrono::duration<double>(steady-map_.received()).count() > cfg_.data_timeout) {
        fail("missing/stale cloud, or cloud frame is not the world frame"); return;
    }
    // A complete new cloud replaces the previous one. Never retain stale obstacles.
    // Aligned PCL2ROS clouds are little-endian float32 XYZ; reject malformed layouts.
    int offsets[3]{-1,-1,-1};
    for (const auto& field : cloud->fields) {
        for (int axis=0; axis<3; ++axis) {
            if (field.name == std::string(1, "xyz"[axis]) &&
                field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count == 1 &&
                field.offset + sizeof(float) <= cloud->point_step)
                offsets[axis] = static_cast<int>(field.offset);
        }
    }
    if (cloud->is_bigendian || offsets[0]<0 || offsets[1]<0 || offsets[2]<0 ||
        cloud->row_step < uint64_t(cloud->width)*cloud->point_step ||
        cloud->data.size() < uint64_t(cloud->row_step)*cloud->height) {
        fail("invalid point cloud XYZ layout"); return;
    }
    std::set<std::pair<int,int>> voxels;
    std::vector<neupan::Vec2> points;
    const auto& p = ctx.robot_state.pose.position;
    std::size_t finite_count = 0;
    for (uint32_t row=0; row<cloud->height; ++row) {
        for (uint32_t col=0; col<cloud->width; ++col) {
            const auto* data = cloud->data.data() + std::size_t(row)*cloud->row_step + std::size_t(col)*cloud->point_step;
            float xyz[3];
            for (int axis=0; axis<3; ++axis) std::memcpy(&xyz[axis], data+offsets[axis], sizeof(float));
            if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) continue;
            ++finite_count;
            // Height is relative to the live base pose, not world zero.
            if (xyz[2]-pose->pose.pose.position.z < cfg_.min_height ||
                xyz[2]-pose->pose.pose.position.z > cfg_.max_height ||
                std::hypot(xyz[0]-p.x, xyz[1]-p.y) > cfg_.max_range) continue;
            const auto key = std::make_pair(int(std::floor((xyz[0]-p.x)/cfg_.voxel_size)),
                                            int(std::floor((xyz[1]-p.y)/cfg_.voxel_size)));
            if (voxels.insert(key).second) points.emplace_back(xyz[0], xyz[1]);
        }
    }
    if (finite_count == 0) { fail("empty or all-invalid cloud"); return; }
    points_.resize(2, points.size());
    for (std::size_t i=0; i<points.size(); ++i) points_.col(i) = points[i];
    ready_ = true;
}

void NeuPANPlanner::configure(const ConfigType& cfg) {
    const auto assets = ament_index_cpp::get_package_share_directory("finenav_example") + "/create_custom_neupan_planner/";
    auto next = std::make_unique<neupan::NeuPANPlanner>(neupan::NeuPANPlanner::fromYaml(
        cfg.config_file.empty() ? assets+"planner.yaml" : cfg.config_file,
        cfg.model_file.empty() ? assets+"models/diff_0p7_0p6.bin" : cfg.model_file));
    if (!(cfg.goal_xy_tolerance > 0) || !(cfg.goal_yaw_tolerance > 0) ||
        !(cfg.collision_margin > 0) || !(cfg.robot_height > 0))
        throw std::invalid_argument("Invalid NeuPAN goal tolerances or robot geometry");
    // hello_finenav runs this layer at 10 Hz.
    if (std::abs(next->config().step_time-0.1) > 1e-6)
        throw std::invalid_argument("NeuPAN step_time must match the 10 Hz control layer (0.1 s)");
    planner_ = std::move(next);
    xy_tolerance_ = cfg.goal_xy_tolerance;
    yaw_tolerance_ = cfg.goal_yaw_tolerance;
    margin_ = cfg.collision_margin;
    reset();
    RCLCPP_INFO(logger_, "NeuPAN C++ ready: footprint %.3f x %.3f m, height %.3f m",
                planner_->config().length, planner_->config().width, cfg.robot_height);
}

void NeuPANPlanner::reset() {
    reference_.clear();
    if (planner_) planner_->reset();
}

bool NeuPANPlanner::collisionFree(const neupan::Vec3& state, const neupan::Vec2& command,
                                 const neupan::Mat2X& points) const {
    const auto& cfg = planner_->config();
    neupan::Vec3 s = state;
    // Swept footprint through command duration AND braking time. Include voxel
    // diagonal (4 cm default) and integration motion allowance in clearance.
    const double duration = 0.1 + std::max(std::abs(command.x())/cfg.max_acce.x(),
                                          std::abs(command.y())/cfg.max_acce.y());
    const int steps = static_cast<int>(std::ceil(duration/0.02));
    for (int step=0; step<=steps; ++step) {
        const double c=std::cos(s.z()), sn=std::sin(s.z());
        for (Eigen::Index i=0; i<points.cols(); ++i) {
            const double dx=points(0,i)-s.x(), dy=points(1,i)-s.y();
            const double bx=c*dx+sn*dy-cfg.wheelbase/2.0, by=-sn*dx+c*dy;
            const double distance=std::hypot(std::max(std::abs(bx)-cfg.length/2.0,0.0),
                                             std::max(std::abs(by)-cfg.width/2.0,0.0));
            if (distance <= margin_ + 0.06) return false;
        }
        s.x() += 0.02*command.x()*c;
        s.y() += 0.02*command.x()*sn;
        s.z() += 0.02*command.y();
    }
    return true;
}

NeuPANPlanner::Result NeuPANPlanner::plan(const Context& ctx, const IWorldView& view) {
    core::OutputTrajectoryT<NeuPANPlanner> result;
    result.twists.emplace_back();  // Explicit stop on every rejected cycle.
    auto& cmd = result.twists.back();
    if (!planner_ || !view.ready() || ctx.ref_traj.poses.empty() ||
        !validPose(ctx.robot_state.pose) || !view.points().allFinite()) {
        reset(); return result;
    }
    for (const auto& pose : ctx.ref_traj.poses) if (!validPose(pose)) { reset(); return result; }
    const auto& p=ctx.robot_state.pose;
    const auto& goal=ctx.ref_traj.poses.back();
    neupan::Vec3 state(p.position.x, p.position.y, yaw(p.orientation));
    neupan::Vec2 action=neupan::Vec2::Zero();
    try {
        if (reference_ != ctx.ref_traj.poses) {
            reference_ = ctx.ref_traj.poses;
            planner_->reset();
            std::vector<neupan::Vec3> waypoints{state};
            for (const auto& pose : reference_) {
                neupan::Vec3 wp(pose.position.x,pose.position.y,yaw(pose.orientation));
                if ((wp.head<2>()-waypoints.back().head<2>()).norm()>1e-5) waypoints.push_back(wp);
            }
            if (waypoints.size()>1) planner_->setWaypoints(waypoints);
        }
        if (std::hypot(goal.position.x-state.x(), goal.position.y-state.y()) <= xy_tolerance_) {
            const double error=wrap(yaw(goal.orientation)-state.z());
            if (std::abs(error)>yaw_tolerance_) {
                action.y()=std::clamp(1.5*error,-planner_->config().max_speed.y(),planner_->config().max_speed.y());
                const double previous = ctx.robot_state.twist.angular.z;
                if (!std::isfinite(previous)) return result;
                const double delta = planner_->config().max_acce.y()*0.1;
                action.y() = std::clamp(action.y(), previous-delta, previous+delta);
            }
        } else {
            neupan::NeuPANPlanner::Info info;
            action=planner_->forward(state,view.points(),info);
            if (!info.solved || info.stop || info.arrive || !action.allFinite()) {
                reset(); return result;
            }
        }
        // The core's acceleration constraints cover adjacent horizon controls;
        // bridge the first command to the measured chassis velocity here.
        const auto& measured = ctx.robot_state.twist;
        if (!std::isfinite(measured.linear.x) || !std::isfinite(measured.angular.z)) {
            reset(); return result;
        }
        const auto delta = planner_->config().max_acce * 0.1;
        action.x()=std::clamp(action.x(), measured.linear.x-delta.x(), measured.linear.x+delta.x());
        action.y()=std::clamp(action.y(), measured.angular.z-delta.y(), measured.angular.z+delta.y());
        const auto limits=planner_->config().max_speed;
        action.x()=std::clamp(action.x(),-limits.x(),limits.x());
        action.y()=std::clamp(action.y(),-limits.y(),limits.y());
        if (!collisionFree(state,action,view.points())) { reset(); return result; }
        cmd.linear.x=action.x(); cmd.angular.z=action.y();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "NeuPAN planning failed; zero velocity: %s", e.what());
        reset();
    }
    return result;
}
}  // namespace finenav::example

FINENAV_REGISTER_MAP(finenav::example::NeuPANCloudMap, neupan_cloud_map)
FINENAV_REGISTER_PLANNER(finenav::example::NeuPANPlanner, neupan_planner)
FINENAV_REGISTER_WORLDVIEW(finenav::example::NeuPANWorldView, neupan_world_view,
    finenav::example::NeuPANPlanner, finenav::example::NeuPANCloudMap)
