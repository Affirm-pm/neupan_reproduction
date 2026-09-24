/*
 * neupan_cpp_ros: ROS2 wrapper for libneupan, the C++ port of NeuPAN.
 *
 * Ported from neupan_ros (https://github.com/hanruihua/neupan_ros),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "msg_utils.hpp"
#include "neupan/log.hpp"
#include "neupan/neupan_planner.hpp"
#include "neupan/safety_supervisor.hpp"

namespace {

using neupan::InitialPath;
using neupan::NeuPANPlanner;
using neupan_ros::planarPose;
using neupan_ros::quatToYaw;

struct NodeParams {
  std::string config_file, dune_checkpoint;
  std::string map_frame, base_frame;
  std::string lidar_frame;  // empty: the frame_id of each scan
  double control_rate;
  double marker_size, marker_z;
  double scan_angle_min, scan_angle_max, scan_range_min, scan_range_max;
  int scan_downsample;
  bool flip_angle;
  bool include_initial_path_direction;
  neupan::SafetySupervisor::Options safety;

  static NodeParams declare(rclcpp::Node& n) {
    NodeParams p;
    p.config_file = n.declare_parameter<std::string>("config_file", "");
    p.dune_checkpoint = n.declare_parameter<std::string>("dune_checkpoint", "");
    p.map_frame = n.declare_parameter<std::string>("map_frame", "map");
    p.base_frame = n.declare_parameter<std::string>("base_frame", "base_link");
    p.lidar_frame = n.declare_parameter<std::string>("lidar_frame", "");
    p.control_rate = n.declare_parameter<double>("control_rate", 50.0);
    p.marker_size = n.declare_parameter<double>("marker_size", 0.05);
    p.marker_z = n.declare_parameter<double>("marker_z", 1.0);
    p.scan_angle_min = n.declare_parameter<double>("scan_angle_min", -3.14);
    p.scan_angle_max = n.declare_parameter<double>("scan_angle_max", 3.14);
    p.scan_range_min = n.declare_parameter<double>("scan_range_min", 0.0);
    p.scan_range_max = n.declare_parameter<double>("scan_range_max", 5.0);
    p.scan_downsample = n.declare_parameter<int>("scan_downsample", 1);
    p.flip_angle = n.declare_parameter<bool>("flip_angle", false);
    p.include_initial_path_direction =
        n.declare_parameter<bool>("include_initial_path_direction", false);
    p.safety.scan_timeout = n.declare_parameter<double>("scan_timeout", 0.5);
    p.safety.solver_fail_grace =
        n.declare_parameter<int>("solver_fail_grace", 5);
    p.safety.stall_speed = n.declare_parameter<double>("stall_speed", 0.02);
    p.safety.stall_timeout = n.declare_parameter<double>("stall_timeout", 3.0);

    if (p.config_file.empty())
      throw std::runtime_error("parameter 'config_file' is required");
    if (p.dune_checkpoint.empty())
      throw std::runtime_error(
          "parameter 'dune_checkpoint' (.bin model) is required");
    if (p.scan_downsample < 1)
      throw std::runtime_error("scan_downsample must be >= 1");
    if (!(p.control_rate > 0.0))
      throw std::runtime_error("control_rate must be > 0");
    return p;
  }
};

std_msgs::msg::ColorRGBA rgb(int r, int g, int b) {
  std_msgs::msg::ColorRGBA c;
  c.r = r / 255.0f;
  c.g = g / 255.0f;
  c.b = b / 255.0f;
  c.a = 1.0f;
  return c;
}

}  // namespace

class NeuPANNode : public rclcpp::Node {
 public:
  NeuPANNode()
      : Node("neupan_node"),
        params_(NodeParams::declare(*this)),
        supervisor_(params_.safety) {
    neupan::setWarningHandler([logger = get_logger()](const std::string& m) {
      RCLCPP_WARN(logger, "%s", m.c_str());
    });
    planner_ = std::make_unique<NeuPANPlanner>(
        NeuPANPlanner::fromYaml(params_.config_file, params_.dune_checkpoint));

    // Relative names, so a namespace or remap moves the whole interface.
    vel_pub_ =
        create_publisher<geometry_msgs::msg::Twist>("neupan_cmd_vel", 10);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("neupan_plan", 10);
    ref_state_pub_ =
        create_publisher<nav_msgs::msg::Path>("neupan_ref_state", 10);
    // Latched and published only when the path changes.
    ref_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "neupan_initial_path", rclcpp::QoS(1).transient_local());
    dune_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "dune_point_markers", 10);
    nrmp_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "nrmp_point_markers", 10);
    robot_marker_pub_ =
        create_publisher<visualization_msgs::msg::Marker>("robot_marker", 10);
    // Published every cycle: a decision layer may drive its chassis off it.
    arrive_pub_ = create_publisher<std_msgs::msg::Bool>("neupan_arrive", 10);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "neupan_diagnostics", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          scanCallback(*msg);
        });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "initial_path", 10, [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
          pathCallback(*msg);
        });
    waypoints_sub_ = create_subscription<nav_msgs::msg::Path>(
        "neupan_waypoints", 10,
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
          waypointsCallback(*msg);
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "neupan_goal", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          goalCallback(*msg);
        });

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / params_.control_rate),
        [this] { run(); });

    RCLCPP_INFO(get_logger(), "neupan_cpp node ready (config: %s)",
                params_.config_file.c_str());
  }

  ~NeuPANNode() override { neupan::setWarningHandler(nullptr); }

 private:
  // ---- control cycle ----

  void run() {
    const rclcpp::Time stamp = now();

    const auto state = lookupPose(params_.base_frame, tf2::TimePointZero);
    if (!state) {
      halt("no robot pose (TF map->base), holding still", stamp);
      return;
    }
    robot_state_ = *state;
    have_state_ = true;

    if (supervisor_.scanStale(stamp.seconds())) {
      obstacle_points_.resize(2, 0);
      halt("scan is stale, obstacles unknown, holding still", stamp);
      return;
    }

    if (!planner_->ensurePath(robot_state_)) {
      halt("waiting for neupan initial path", stamp);
      return;
    }
    publishInitialPathIfChanged(stamp);

    if (obstacle_points_.cols() == 0)
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "no obstacle points, only path tracking will be performed");

    NeuPANPlanner::Info info;
    const neupan::Vec2 action =
        planner_->forward(robot_state_, obstacle_points_, info);
    const neupan::Vec2 cmd = supervisor_.filter(info, action, stamp.seconds());
    logCycle(info);

    publishCommand(cmd);
    publishArrive(info.arrive);
    publishDiagnostics(info, cmd, stamp);
    publishVisualization(info, stamp);
  }

  // Publishing nothing is not a stop: consumers hold the last command.
  void halt(const char* why, const rclcpp::Time& stamp) {
    publishCommand(neupan::Vec2::Zero());
    publishArrive(false);
    publishStatus(diagnostic_msgs::msg::DiagnosticStatus::WARN, why, {}, stamp);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s", why);
  }

  void logCycle(const NeuPANPlanner::Info& info) {
    if (info.arrive)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "arrive at the target");
    if (info.stop)
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "neupan stop, min distance %.3f below threshold %.3f",
          info.min_distance, planner_->collisionThreshold());
    if (supervisor_.solverGaveUp())
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "%d consecutive unsolved cycles (osqp status %d), stopping",
          supervisor_.consecutiveUnsolved(), info.solver_status);
    if (supervisor_.stalled())
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "stalled for %.1f s without arriving; the path may "
                           "be blocked and need a global replan",
                           params_.safety.stall_timeout);
  }

  // Pose of `frame` in the map frame at `time` (TimePointZero: latest).
  std::optional<neupan::Vec3> lookupPose(const std::string& frame,
                                         const tf2::TimePoint& time) {
    try {
      const auto tfs =
          tf_buffer_->lookupTransform(params_.map_frame, frame, time);
      return neupan::Vec3(tfs.transform.translation.x,
                          tfs.transform.translation.y,
                          quatToYaw(tfs.transform.rotation));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "waiting for tf %s -> %s: %s",
                           params_.map_frame.c_str(), frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  // ---- inputs ----

  void scanCallback(const sensor_msgs::msg::LaserScan& msg) {
    if (!have_state_) return;

    // The sensor pose when the scan was taken, so a moving robot does not
    // smear the points; the latest pose if TF has not caught up yet.
    const std::string& frame =
        params_.lidar_frame.empty() ? msg.header.frame_id : params_.lidar_frame;
    const tf2::TimePoint scan_time = tf2_ros::fromMsg(msg.header.stamp);
    std::optional<neupan::Vec3> lidar_pose;
    if (scan_time != tf2::TimePointZero &&
        tf_buffer_->canTransform(params_.map_frame, frame, scan_time))
      lidar_pose = lookupPose(frame, scan_time);
    if (!lidar_pose) lidar_pose = lookupPose(frame, tf2::TimePointZero);
    if (!lidar_pose) return;

    supervisor_.recordScan(now().seconds());

    const int n = static_cast<int>(msg.ranges.size());
    // Deriving the increment from the range divides by one beam too few.
    const double angle_inc =
        msg.angle_increment != 0.0f
            ? msg.angle_increment
            : (n > 1 ? (msg.angle_max - msg.angle_min) / (n - 1) : 0.0);

    const double c = std::cos((*lidar_pose)(2)), s = std::sin((*lidar_pose)(2));
    neupan::Mat2X pts(2, n);
    int kept = 0;
    for (int i = 0; i < n; i += params_.scan_downsample) {
      const double r = msg.ranges[i];
      const double angle = params_.flip_angle ? msg.angle_max - i * angle_inc
                                              : msg.angle_min + i * angle_inc;
      // Written so that NaN ranges (no return) fail the test.
      const bool keep =
          r >= params_.scan_range_min && r <= params_.scan_range_max &&
          angle > params_.scan_angle_min && angle < params_.scan_angle_max;
      if (!keep) continue;
      const double x = r * std::cos(angle), y = r * std::sin(angle);
      pts(0, kept) = c * x - s * y + (*lidar_pose)(0);
      pts(1, kept) = s * x + c * y + (*lidar_pose)(1);
      ++kept;
    }
    pts.conservativeResize(2, kept);
    obstacle_points_ = std::move(pts);
  }

  // Path poses -> [x, y, theta, gear]; theta from the points gradient unless
  // include_initial_path_direction is set (as upstream).
  std::vector<InitialPath::PathPoint> pathMsgToPoints(
      const nav_msgs::msg::Path& msg) const {
    std::vector<InitialPath::PathPoint> out;
    const size_t n = msg.poses.size();
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const auto& p = msg.poses[i].pose;
      double theta;
      if (params_.include_initial_path_direction) {
        theta = quatToYaw(p.orientation);
      } else if (i + 1 < n) {
        const auto& p2 = msg.poses[i + 1].pose;
        theta = std::atan2(p2.position.y - p.position.y,
                           p2.position.x - p.position.x);
      } else {
        theta = out.empty() ? quatToYaw(p.orientation) : out.back()(2);
      }
      out.emplace_back(p.position.x, p.position.y, theta, 1.0);
    }
    return out;
  }

  void pathCallback(const nav_msgs::msg::Path& msg) {
    if (msg.poses.size() < 2) {
      RCLCPP_WARN(get_logger(), "ignoring initial path with < 2 poses");
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "initial path update from given path (%zu poses)",
                         msg.poses.size());

    // Replacing rather than resetting: a resetting replan is a cold start.
    if (planner_->hasPath() && have_state_) {
      planner_->replaceInitialPath(pathMsgToPoints(msg), robot_state_);
    } else {
      planner_->setInitialPath(pathMsgToPoints(msg));
      planner_->reset();
    }
    publishArrive(false);
  }

  void waypointsCallback(const nav_msgs::msg::Path& msg) {
    if (!have_state_ || msg.poses.empty()) return;

    // Identical republished waypoints must not cold-reset the planner.
    const auto pts = pathMsgToPoints(msg);
    if (samePoints(pts, last_waypoints_)) return;
    last_waypoints_ = pts;

    std::vector<neupan::Vec3> wps{robot_state_};
    for (const auto& pp : pts) wps.emplace_back(pp.head<3>());

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "initial path update from waypoints");
    planner_->setWaypoints(wps);
    planner_->reset();
    publishArrive(false);
  }

  static bool samePoints(const std::vector<InitialPath::PathPoint>& a,
                         const std::vector<InitialPath::PathPoint>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
      if ((a[i] - b[i]).cwiseAbs().maxCoeff() > 1e-9) return false;
    return true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped& msg) {
    if (!have_state_) return;
    const neupan::Vec3 goal(msg.pose.position.x, msg.pose.position.y,
                            quatToYaw(msg.pose.orientation));
    RCLCPP_INFO(get_logger(), "set neupan goal: [%.2f, %.2f, %.2f]", goal(0),
                goal(1), goal(2));
    planner_->updateInitialPathFromGoal(robot_state_, goal);
    planner_->reset();
  }

  // ---- outputs ----

  void publishCommand(const neupan::Vec2& cmd) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd(0);
    twist.angular.z = cmd(1);
    vel_pub_->publish(twist);
  }

  void publishArrive(bool arrived) {
    std_msgs::msg::Bool msg;
    msg.data = arrived;
    arrive_pub_->publish(msg);
  }

  void publishStatus(
      uint8_t level, const std::string& message,
      const std::vector<std::pair<std::string, std::string>>& values,
      const rclcpp::Time& stamp) {
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "neupan";
    st.hardware_id = params_.base_frame;
    st.level = level;
    st.message = message;
    for (const auto& [k, v] : values) {
      diagnostic_msgs::msg::KeyValue e;
      e.key = k;
      e.value = v;
      st.values.push_back(e);
    }
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = stamp;
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  void publishDiagnostics(const NeuPANPlanner::Info& info,
                          const neupan::Vec2& cmd, const rclcpp::Time& stamp) {
    using diagnostic_msgs::msg::DiagnosticStatus;
    using Level = neupan::SafetySupervisor::Level;
    const auto status = supervisor_.status(info);
    const uint8_t level = status.level == Level::Error ? DiagnosticStatus::ERROR
                          : status.level == Level::Warn ? DiagnosticStatus::WARN
                                                        : DiagnosticStatus::OK;
    const auto flag = [](bool b) { return b ? "true" : "false"; };

    publishStatus(level, status.message,
                  {{"solved", flag(info.solved)},
                   {"solver_status", std::to_string(info.solver_status)},
                   {"consecutive_unsolved",
                    std::to_string(supervisor_.consecutiveUnsolved())},
                   {"min_distance", std::to_string(info.min_distance)},
                   {"obstacle_points", std::to_string(obstacle_points_.cols())},
                   {"stalled", flag(supervisor_.stalled())},
                   {"cmd_v", std::to_string(cmd(0))},
                   {"cmd_w", std::to_string(cmd(1))}},
                  stamp);
  }

  void publishInitialPathIfChanged(const rclcpp::Time& stamp) {
    const int revision = planner_->ipath().revision();
    if (revision == published_path_revision_) return;
    published_path_revision_ = revision;

    nav_msgs::msg::Path msg;
    msg.header = header(stamp);
    for (const auto& p : planner_->ipath().initialPath())
      msg.poses.push_back(planarPose(msg.header, p(0), p(1), p(2)));
    ref_path_pub_->publish(msg);
  }

  // Visualization only; skipped for topics nobody is listening to.
  void publishVisualization(const NeuPANPlanner::Info& info,
                            const rclcpp::Time& stamp) {
    if (plan_pub_->get_subscription_count() > 0 && info.opt_s.cols() > 0)
      plan_pub_->publish(statesToPath(info.opt_s, stamp));
    if (ref_state_pub_->get_subscription_count() > 0 && info.ref_s.cols() > 0)
      ref_state_pub_->publish(statesToPath(info.ref_s, stamp));
    publishPointMarkers(info.dune_points, *dune_markers_pub_, rgb(160, 32, 240),
                        stamp);
    publishPointMarkers(info.nrmp_points, *nrmp_markers_pub_, rgb(255, 128, 0),
                        stamp);
    if (robot_marker_pub_->get_subscription_count() > 0)
      publishRobotMarker(stamp);
  }

  std_msgs::msg::Header header(const rclcpp::Time& stamp) const {
    std_msgs::msg::Header h;
    h.frame_id = params_.map_frame;
    h.stamp = stamp;
    return h;
  }

  nav_msgs::msg::Path statesToPath(const neupan::Mat3X& states,
                                   const rclcpp::Time& stamp) const {
    nav_msgs::msg::Path path;
    path.header = header(stamp);
    for (Eigen::Index i = 0; i < states.cols(); ++i)
      path.poses.push_back(
          planarPose(path.header, states(0, i), states(1, i), states(2, i)));
    return path;
  }

  // One CUBE_LIST marker replaced every cycle. An empty set deletes it, so
  // points from an earlier cycle never linger in RViz.
  void publishPointMarkers(
      const neupan::Mat2X& points,
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>& pub,
      const std_msgs::msg::ColorRGBA& color, const rclcpp::Time& stamp) {
    if (pub.get_subscription_count() == 0) return;
    visualization_msgs::msg::Marker m;
    m.header = header(stamp);
    m.id = 0;
    if (points.cols() == 0) {
      m.action = visualization_msgs::msg::Marker::DELETE;
    } else {
      m.action = visualization_msgs::msg::Marker::ADD;
      m.type = visualization_msgs::msg::Marker::CUBE_LIST;
      m.scale.x = m.scale.y = m.scale.z = params_.marker_size;
      m.color = color;
      m.pose.orientation.w = 1.0;
      m.points.resize(points.cols());
      for (Eigen::Index i = 0; i < points.cols(); ++i) {
        m.points[i].x = points(0, i);
        m.points[i].y = points(1, i);
        m.points[i].z = 0.3;
      }
    }
    visualization_msgs::msg::MarkerArray arr;
    arr.markers.push_back(std::move(m));
    pub.publish(arr);
  }

  void publishRobotMarker(const rclcpp::Time& stamp) {
    const auto& cfg = planner_->config();
    visualization_msgs::msg::Marker m;
    m.header = header(stamp);
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.color = rgb(0, 255, 0);
    m.scale.x = cfg.length;
    m.scale.y = cfg.width;
    m.scale.z = params_.marker_z;
    m.pose =
        planarPose(m.header, robot_state_(0), robot_state_(1), robot_state_(2))
            .pose;
    robot_marker_pub_->publish(m);
  }

  // ---- state ----

  const NodeParams params_;
  neupan::SafetySupervisor supervisor_;
  std::unique_ptr<NeuPANPlanner> planner_;

  neupan::Vec3 robot_state_ = neupan::Vec3::Zero();
  bool have_state_ = false;
  neupan::Mat2X obstacle_points_ = neupan::Mat2X(2, 0);
  std::vector<InitialPath::PathPoint> last_waypoints_;
  int published_path_revision_ = -1;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_, ref_state_pub_,
      ref_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      dune_markers_pub_,
      nrmp_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arrive_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      robot_marker_pub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_,
      waypoints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NeuPANNode>());
  rclcpp::shutdown();
  return 0;
}
