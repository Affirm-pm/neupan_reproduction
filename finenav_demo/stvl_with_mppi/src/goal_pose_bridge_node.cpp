// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

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

} // namespace

class GoalPoseLinePathTestNode : public rclcpp::Node {
public:
    using ComputePlan = finenav_msgs::action::ComputePlan;
    using GoalHandleComputePlan = rclcpp_action::ClientGoalHandle<ComputePlan>;

    GoalPoseLinePathTestNode()
    : rclcpp::Node("goal_pose_bridge_node")
    {
        constexpr const char* kGoalTopic        = "/goal_pose";
        constexpr const char* kPathTopic        = "/hello_finenav/line_path";
        constexpr const char* kNavStateTopic    = "/nav_state";
        constexpr const char* kMapFrame         = "map";
        constexpr const char* kActionServer     = "/hello_finenav/mppi_layer";
        constexpr const char* kPlannerId        = "finenav_mppi_controller";
        constexpr const char* kMppiPlanTopic    = "/hello_finenav/mppi_layer_plan";
        constexpr const char* kNavPathTopic     = "/mppi_path";
        constexpr const char* kLinePathNavTopic = "/line_path_nav";
        constexpr int kNumPoints = 50;

        path_topic_          = kPathTopic;
        map_frame_           = kMapFrame;
        action_server_name_  = kActionServer;
        planner_id_          = kPlannerId;
        mppi_plan_topic_     = kMppiPlanTopic;
        nav_path_topic_      = kNavPathTopic;
        line_path_nav_topic_ = kLinePathNavTopic;
        num_points_          = kNumPoints;

        // 发布者
        path_pub_          = this->create_publisher<finenav_msgs::msg::Trajectory>(path_topic_, 10);
        nav_path_pub_      = this->create_publisher<nav_msgs::msg::Path>(nav_path_topic_, 10);
        line_path_nav_pub_ = this->create_publisher<nav_msgs::msg::Path>(line_path_nav_topic_, 10);

        // Action 客户端
        action_client_ = rclcpp_action::create_client<ComputePlan>(this, action_server_name_);

        // 订阅 nav_state 获取机器人位姿（替代 TF lookup）
        nav_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            kNavStateTopic, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(pose_mutex_);
                latest_pose_ = msg->pose.pose;
                has_pose_ = true;
            });

        // 直接订阅 /goal_pose（不再经过 MessageFilter）
        goal_sub_raw_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            kGoalTopic, 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                this->onGoal(msg);
            });

        // 订阅 MPPI 规划结果
        mppi_plan_sub_ = this->create_subscription<finenav_msgs::msg::Trajectory>(
            mppi_plan_topic_, 10,
            std::bind(&GoalPoseLinePathTestNode::mppiPlanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Ready. goal_topic=%s  path_topic=%s  action_server=%s  planner_id=%s",
                    kGoalTopic, path_topic_.c_str(),
                    action_server_name_.c_str(), planner_id_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "nav_state topic=%s  mppi_plan topic=%s  nav_path topic=%s",
                    kNavStateTopic, mppi_plan_topic_.c_str(), nav_path_topic_.c_str());
    }

private:
    void mppiPlanCallback(const finenav_msgs::msg::Trajectory::SharedPtr msg)
    {
        if (!(msg->valid_fields & TRAJ_POSE) || msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "MPPI plan has no pose data, skipping");
            return;
        }
        nav_path_pub_->publish(convertTrajectoryToPath(*msg));
    }

    nav_msgs::msg::Path convertTrajectoryToPath(const finenav_msgs::msg::Trajectory& traj)
    {
        nav_msgs::msg::Path path;
        path.header.stamp    = this->now();
        path.header.frame_id = map_frame_;
        path.poses.reserve(traj.poses.size());
        for (size_t i = 0; i < traj.poses.size(); ++i) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp    = path.header.stamp;
            ps.header.frame_id = map_frame_;
            ps.pose            = traj.poses[i];
            path.poses.push_back(ps);
        }
        return path;
    }

    void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 从缓存的 nav_state 获取机器人起始位姿
        geometry_msgs::msg::Pose start_pose;
        {
            std::lock_guard<std::mutex> lk(pose_mutex_);
            if (!has_pose_) {
                RCLCPP_WARN(this->get_logger(),
                            "No nav_state received yet, cannot compute start pose. "
                            "Waiting for /nav_state ...");
                return;
            }
            start_pose = latest_pose_;
        }

        const double sx = start_pose.position.x;
        const double sy = start_pose.position.y;
        const double sz = start_pose.position.z;
        const double gx = msg->pose.position.x;
        const double gy = msg->pose.position.y;
        const double gz = msg->pose.position.z;

        RCLCPP_INFO(this->get_logger(),
                    "Goal received: start=(%.2f, %.2f) -> goal=(%.2f, %.2f)",
                    sx, sy, gx, gy);

        const double dx = gx - sx, dy = gy - sy, dz = gz - sz;
        if (std::sqrt(dx*dx + dy*dy + dz*dz) < 1e-6) {
            RCLCPP_WARN(this->get_logger(), "Start and goal coincide, skipping");
            return;
        }

        // 生成直线参考轨迹并发布
        auto ref_traj = generateStraightLineTrajectory(sx, sy, sz, gx, gy, gz);
        path_pub_->publish(ref_traj);
        line_path_nav_pub_->publish(convertTrajectoryToPath(ref_traj));
        RCLCPP_INFO(this->get_logger(),
                    "Published reference trajectory: %zu points", ref_traj.poses.size());

        // 发送 Action Goal
        sendActionGoal(msg->pose);
    }

    finenav_msgs::msg::Trajectory generateStraightLineTrajectory(
        double sx, double sy, double sz,
        double gx, double gy, double gz)
    {
        auto traj = finenav_msgs::msg::Trajectory();
        traj.header.stamp    = this->now();
        traj.header.frame_id = map_frame_;
        traj.valid_fields    = TRAJ_POSE;
        traj.poses.reserve(num_points_);

        const double dx  = gx - sx;
        const double dy  = gy - sy;
        const double dz  = gz - sz;
        const double yaw = std::atan2(dy, dx);
        const auto   q   = quatFromYaw(yaw);

        // 测试模式：仅发布终点单点，此时PathFollow和PathAlign Crititc 实际无效
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x  = gx;
            pose.position.y  = gy;
            pose.position.z  = gz;
            pose.orientation = q; // TODO: 应该使用goal_pose朝向才对，无所谓了
            traj.poses.push_back(pose);
        }
        // 完整直线路径（多点，用于正式导航）：
        // for (int i = 0; i < num_points_; ++i) {
        //     const double t = static_cast<double>(i) / static_cast<double>(num_points_ - 1);
        //     geometry_msgs::msg::Pose pose;
        //     pose.position.x  = sx + t * dx;
        //     pose.position.y  = sy + t * dy;
        //     pose.position.z  = sz + t * dz;
        //     pose.orientation = q;
        //     traj.poses.push_back(pose);
        // }
        return traj;
    }

    void sendActionGoal(const geometry_msgs::msg::Pose& goal_pose)
    {
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal_msg        = ComputePlan::Goal();
        goal_msg.planner_id  = planner_id_;
        goal_msg.goal        = goal_pose;

        RCLCPP_INFO(this->get_logger(),
                    "Sending action goal: planner_id=%s target=(%.2f, %.2f, %.2f)",
                    planner_id_.c_str(),
                    goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);

        auto opts = rclcpp_action::Client<ComputePlan>::SendGoalOptions();

        opts.goal_response_callback =
            [this](GoalHandleComputePlan::SharedPtr gh) {
                if (!gh) { RCLCPP_ERROR(this->get_logger(), "Goal rejected"); return; }
                goal_handle_ = gh;
                RCLCPP_INFO(this->get_logger(), "Goal accepted");
            };

        opts.result_callback =
            [this](const GoalHandleComputePlan::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Plan succeeded! time=%.3fs",
                                    rclcpp::Duration(result.result->planning_time).seconds());
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Plan aborted"); break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "Plan canceled"); break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d",
                                     static_cast<int>(result.code));
                }
                goal_handle_.reset();
            };

        action_client_->async_send_goal(goal_msg, opts);
    }

    // ---------- publishers ----------
    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr line_path_nav_pub_;

    // ---------- subscribers ----------
    rclcpp::Subscription<finenav_msgs::msg::Trajectory>::SharedPtr mppi_plan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_raw_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_state_sub_;

    // ---------- action ----------
    rclcpp_action::Client<ComputePlan>::SharedPtr action_client_;
    GoalHandleComputePlan::SharedPtr goal_handle_;

    // ---------- robot pose cache ----------
    std::mutex pose_mutex_;
    geometry_msgs::msg::Pose latest_pose_{};
    bool has_pose_{false};

    // ---------- params ----------
    std::string path_topic_;
    std::string map_frame_;
    std::string action_server_name_;
    std::string planner_id_;
    std::string mppi_plan_topic_;
    std::string nav_path_topic_;
    std::string line_path_nav_topic_;
    int num_points_{50};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoseLinePathTestNode>());
    rclcpp::shutdown();
    return 0;
}