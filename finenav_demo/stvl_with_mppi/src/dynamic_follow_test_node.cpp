// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>
#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "finenav_msgs/action/navigate_to_pose.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

using std::chrono_literals::operator""ms;

namespace {

constexpr uint32_t TRAJ_POSE = 1;

geometry_msgs::msg::Quaternion quatFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(0.5 * yaw);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  return q;
}

double yawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

/**
 * @brief 动态跟随测试节点。
 *
 * 功能:
 *  1. 向 Navigator 发送 NavigateToPose action，指定 behavior_tree="DynamicFollowPose"
 *  2. 以圆形轨迹持续发布 /target_pose，模拟不断移动的目标
 *  3. 同步发布 /hello_finenav/line_path 参考轨迹，供 ControlLayer 使用
 *
 * 参数（可在 launch 文件中通过 parameters 传入）:
 *   - circle_radius   (double, 默认 3.0 m):  目标圆形轨迹半径
 *   - circle_period   (double, 默认 30.0 s): 目标绕圈周期
 *   - follow_distance (double, 默认 1.0 m):  跟随距离（与 DynamicGoalFollower 保持一致）
 *   - target_height   (double, 默认 0.0 m):  目标点 Z 坐标
 *   - circle_center_x (double, 默认 0.0 m):  圆心 X
 *   - circle_center_y (double, 默认 0.0 m):  圆心 Y
 *   - line_path_points (int,   默认 30):     参考轨迹点数
 */
class DynamicFollowTestNode : public rclcpp::Node
{
public:
  using NavigateToPose     = finenav_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  DynamicFollowTestNode()
  : rclcpp::Node("dynamic_follow_test_node")
  {
    // ---------- 参数 ----------
    this->declare_parameter<double>("circle_radius", 3.0);
    this->declare_parameter<double>("circle_period", 30.0);
    this->declare_parameter<double>("follow_distance", 1.0);
    this->declare_parameter<double>("target_height", 0.0);
    this->declare_parameter<double>("circle_center_x", 0.0);
    this->declare_parameter<double>("circle_center_y", 0.0);
    this->declare_parameter<int>("line_path_points", 30);

    circle_radius_   = this->get_parameter("circle_radius").as_double();
    circle_period_   = this->get_parameter("circle_period").as_double();
    follow_distance_ = this->get_parameter("follow_distance").as_double();
    target_height_   = this->get_parameter("target_height").as_double();
    circle_center_x_ = this->get_parameter("circle_center_x").as_double();
    circle_center_y_ = this->get_parameter("circle_center_y").as_double();
    line_path_points_ = static_cast<size_t>(this->get_parameter("line_path_points").as_int());

    // ---------- 发布者 ----------
    target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);
    line_path_pub_   = this->create_publisher<finenav_msgs::msg::Trajectory>(
        "/hello_finenav/line_path", 10);

    // ---------- 订阅者 ----------
    nav_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/nav_state", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(pose_mutex_);
          latest_pose_ = msg->pose.pose;
          has_pose_    = true;
        });

    // ---------- Action 客户端 ----------
    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "/hello_finenav/navigate_to_pose");

    // ---------- 计时器（发布 /target_pose）----------
    const auto period_ms = static_cast<int>(std::max(20.0, circle_period_ * 1000.0 / 200.0));
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&DynamicFollowTestNode::publishTarget, this));

    // ---------- 延迟启动 action ----------
    start_timer_ = this->create_wall_timer(
        3000ms,
        [this]() {
          start_timer_->cancel();
          sendNavigateGoal();
        });

    RCLCPP_INFO(this->get_logger(),
                "DynamicFollowTestNode ready. circle(%.1f m, %.1f s), follow_dist=%.1f m",
                circle_radius_, circle_period_, follow_distance_);
  }

private:
  // ──────────────────────────────────────────────
  // 计算圆形轨迹上的目标位姿
  // ──────────────────────────────────────────────
  geometry_msgs::msg::Pose computeTargetPose() const
  {
    const double now_sec = this->now().seconds();
    const double ang_vel = 2.0 * M_PI / circle_period_;
    const double angle   = ang_vel * now_sec;

    geometry_msgs::msg::Pose target;
    target.position.x  = circle_center_x_ + circle_radius_ * std::cos(angle);
    target.position.y  = circle_center_y_ + circle_radius_ * std::sin(angle);
    target.position.z  = target_height_;
    target.orientation = quatFromYaw(angle + M_PI_2);  // heading 沿切线方向
    return target;
  }

  // ──────────────────────────────────────────────
  // 根据 target & robot 计算跟随目标（与 DynamicGoalFollower 逻辑一致）
  // ──────────────────────────────────────────────
  geometry_msgs::msg::Pose computeFollowGoal(
      const geometry_msgs::msg::Pose& robot_pose,
      const geometry_msgs::msg::Pose& target_pose) const
  {
    auto goal = target_pose;
    const double dx   = target_pose.position.x - robot_pose.position.x;
    const double dy   = target_pose.position.y - robot_pose.position.y;
    const double dist = std::hypot(dx, dy);

    if (follow_distance_ > 0.0 && dist > follow_distance_) {
      const double scale = (dist - follow_distance_) / dist;
      goal.position.x = robot_pose.position.x + dx * scale;
      goal.position.y = robot_pose.position.y + dy * scale;
    }

    goal.orientation = quatFromYaw(std::atan2(dy, dx));
    return goal;
  }

  // ──────────────────────────────────────────────
  // 生成直线参考轨迹
  // ──────────────────────────────────────────────
  finenav_msgs::msg::Trajectory generateLineTrajectory(
      const geometry_msgs::msg::Pose& start,
      const geometry_msgs::msg::Pose& end) const
  {
    auto traj = finenav_msgs::msg::Trajectory();
    traj.header.stamp    = this->now();
    traj.header.frame_id = "map";
    traj.valid_fields    = TRAJ_POSE;
    traj.poses.reserve(line_path_points_);

    const double dx  = end.position.x - start.position.x;
    const double dy  = end.position.y - start.position.y;
    const double dz  = end.position.z - start.position.z;
    const double yaw = std::atan2(dy, dx);
    const auto   q   = quatFromYaw(yaw);

    for (size_t i = 0; i < line_path_points_; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(line_path_points_ - 1);
      geometry_msgs::msg::Pose pose;
      pose.position.x  = start.position.x + t * dx;
      pose.position.y  = start.position.y + t * dy;
      pose.position.z  = start.position.z + t * dz;
      pose.orientation = q;
      traj.poses.push_back(pose);
    }
    return traj;
  }

  // ──────────────────────────────────────────────
  // 发布 /target_pose + /hello_finenav/line_path
  // ──────────────────────────────────────────────
  void publishTarget()
  {
    const auto target_pose = computeTargetPose();

    // 1. 发布目标位姿（DynamicGoalFollower 订阅）
    {
      auto msg = geometry_msgs::msg::PoseStamped();
      msg.header.stamp    = this->now();
      msg.header.frame_id = "map";
      msg.pose            = target_pose;
      target_pose_pub_->publish(msg);
    }

    // 2. 发布参考轨迹（ControlLayer prePlan 读取）
    {
      geometry_msgs::msg::Pose robot_pose;
      {
        std::lock_guard<std::mutex> lk(pose_mutex_);
        if (!has_pose_) {
          return;  // 还没有位姿，等下一帧
        }
        robot_pose = latest_pose_;
      }
      const auto follow_goal = computeFollowGoal(robot_pose, target_pose);
      line_path_pub_->publish(generateLineTrajectory(robot_pose, follow_goal));
    }
  }

  // ──────────────────────────────────────────────
  // 发送 NavigateToPose action，启动 DynamicFollowPose 行为树
  // ──────────────────────────────────────────────
  void sendNavigateGoal()
  {
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(),
                   "NavigateToPose action server not available, retrying...");
      // 重试
      start_timer_ = this->create_wall_timer(
          2000ms, [this]() { start_timer_->cancel(); sendNavigateGoal(); });
      return;
    }

    // 初始 goal：用圆形轨迹上的当前位置作为初始目标
    const auto initial_target = computeTargetPose();

    NavigateToPose::Goal goal_msg;
    goal_msg.pose.pose      = initial_target;
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = this->now();
    goal_msg.behavior_tree  = "DynamicFollowPose";

    RCLCPP_INFO(this->get_logger(),
                "Sending NavigateToPose goal (%.2f, %.2f)  bt='DynamicFollowPose'",
                initial_target.position.x, initial_target.position.y);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    opts.goal_response_callback =
        [this](const GoalHandleNavigate::SharedPtr& gh) {
          if (!gh) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server.");
            return;
          }
          active_goal_handle_ = gh;
          RCLCPP_INFO(this->get_logger(), "DynamicFollowPose goal accepted.");
        };

    opts.result_callback =
        [this](const GoalHandleNavigate::WrappedResult& result) {
          active_goal_handle_ = nullptr;
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(this->get_logger(), "DynamicFollow finished (goal reached).");
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_INFO(this->get_logger(), "DynamicFollow canceled.");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_WARN(this->get_logger(), "DynamicFollow aborted.");
              break;
            default:
              RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
          }
        };

    nav_action_client_->async_send_goal(goal_msg, opts);
  }

  // ────────── 参数 ──────────
  double circle_radius_   = 3.0;
  double circle_period_   = 30.0;
  double follow_distance_ = 1.0;
  double target_height_   = 0.0;
  double circle_center_x_ = 0.0;
  double circle_center_y_ = 0.0;
  size_t line_path_points_ = 30;

  // ────────── 发布者 ──────────
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr   line_path_pub_;

  // ────────── 订阅者 ──────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_state_sub_;

  // ────────── Action ──────────
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  GoalHandleNavigate::SharedPtr active_goal_handle_;

  // ────────── 位姿缓存 ──────────
  std::mutex pose_mutex_;
  geometry_msgs::msg::Pose latest_pose_{};
  bool has_pose_{false};

  // ────────── 计时器 ──────────
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicFollowTestNode>());
  rclcpp::shutdown();
  return 0;
}
