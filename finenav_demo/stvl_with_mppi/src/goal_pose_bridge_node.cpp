// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <message_filters/subscriber.h>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

namespace {

// Trajectory 位掩码常量（与消息定义一致）
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
        // 硬编码参数
        constexpr const char* kGoalTopic = "/goal_pose";
        constexpr const char* kPathTopic = "/hello_finenav/line_path";
        constexpr const char* kMapFrame = "map";
        constexpr const char* kBaseFrame = "base_link";
        constexpr const char* kActionServer = "/hello_finenav/mppi_layer";
        constexpr const char* kPlannerId = "finenav_mppi_controller";
        constexpr const char* kMppiPlanTopic = "/hello_finenav/mppi_layer_plan";
        constexpr const char* kNavPathTopic = "/mppi_path";
        constexpr int kNumPoints = 50;

        goal_topic_ = kGoalTopic;
        path_topic_ = kPathTopic;
        map_frame_ = kMapFrame;
        base_frame_ = kBaseFrame;
        action_server_name_ = kActionServer;
        planner_id_ = kPlannerId;
        mppi_plan_topic_ = kMppiPlanTopic;
        nav_path_topic_ = kNavPathTopic;
        num_points_ = kNumPoints;

        // 创建 Trajectory 发布者（参考轨迹）
        path_pub_ = this->create_publisher<finenav_msgs::msg::Trajectory>(path_topic_, 10);

        // 创建 Action 客户端
        action_client_ = rclcpp_action::create_client<ComputePlan>(this, action_server_name_);

        // ========== 新增：订阅 MPPI 规划结果 ==========
        mppi_plan_sub_ = this->create_subscription<finenav_msgs::msg::Trajectory>(
            mppi_plan_topic_,
            10,
            std::bind(&GoalPoseLinePathTestNode::mppiPlanCallback, this, std::placeholders::_1));

        // ========== 新增：发布转换后的 nav_msgs::Path ==========
        nav_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(nav_path_topic_, 10);

        // TF 缓冲区
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setCreateTimerInterface(
            std::make_shared<tf2_ros::CreateTimerROS>(
                this->get_node_base_interface(),
                this->get_node_timers_interface()));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // MessageFilter 订阅
        goal_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
            this, goal_topic_);

        goal_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>(
            *goal_sub_,
            *tf_buffer_,
            map_frame_,
            10,
            this->get_node_logging_interface(),
            this->get_node_clock_interface(),
            10ms);

        goal_filter_->registerCallback(
            [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
                this->onGoal(msg);
            });

        RCLCPP_INFO(this->get_logger(),
                    "Ready. goal_topic=%s path_topic=%s action_server=%s planner_id=%s",
                    goal_topic_.c_str(), path_topic_.c_str(),
                    action_server_name_.c_str(), planner_id_.c_str());
        RCLCPP_INFO(this->get_logger(),
                    "Subscribing to MPPI plan: %s, publishing nav path to: %s",
                    mppi_plan_topic_.c_str(), nav_path_topic_.c_str());
    }

private:
    // ========== 新增：MPPI 规划结果回调 ==========
    void mppiPlanCallback(const finenav_msgs::msg::Trajectory::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received MPPI plan: valid_fields=0x%X, poses=%zu",
                    msg->valid_fields, msg->poses.size());

        // 检查是否有位置信息
        if (!(msg->valid_fields & TRAJ_POSE) || msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "MPPI plan has no pose data, skipping conversion");
            return;
        }

        // 转换为 nav_msgs::Path
        auto path_msg = convertTrajectoryToPath(*msg);

        // 发布
        nav_path_pub_->publish(path_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published nav path: %zu poses, frame_id=%s",
                    path_msg.poses.size(), path_msg.header.frame_id.c_str());
    }

    // ========== 新增：Trajectory 转 nav_msgs::Path ==========
    nav_msgs::msg::Path convertTrajectoryToPath(const finenav_msgs::msg::Trajectory& traj)
    {
        nav_msgs::msg::Path path;

        // 复制头部信息
        path.header.stamp = this->now();
        path.header.frame_id = "map";

        // 转换每个位姿点
        path.poses.reserve(traj.poses.size());

        for (size_t i = 0; i < traj.poses.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = traj.header.stamp;
            pose_stamped.header.frame_id = traj.header.frame_id;
            pose_stamped.pose = traj.poses[i];

            // 如果有时间信息，可以使用每个点的独立时间戳
            if (traj.valid_fields & 8 && i < traj.time_from_start.size()) {
                // 注意：这里的时间是 from_start，需要转换为绝对时间
                // 简化处理：保持相同的时间戳
            }

            path.poses.push_back(pose_stamped);
        }

        return path;
    }

    void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received goal: (%.2f, %.2f, %.2f)",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        // 1) 获取当前机器人位姿
        geometry_msgs::msg::TransformStamped base_tf;
        try {
            base_tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF lookup map->base_link failed: %s", ex.what());
            return;
        }

        const double sx = base_tf.transform.translation.x;
        const double sy = base_tf.transform.translation.y;
        const double sz = base_tf.transform.translation.z;

        const double gx = msg->pose.position.x;
        const double gy = msg->pose.position.y;
        const double gz = msg->pose.position.z;

        const double dx = gx - sx;
        const double dy = gy - sy;
        const double dz = gz - sz;
        const double total_distance = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (total_distance < 1e-6) {
            RCLCPP_WARN(this->get_logger(), "Start and goal coincide, skipping");
            return;
        }

        // 2) 生成参考轨迹并发布
        auto ref_traj = generateStraightLineTrajectory(sx, sy, sz, gx, gy, gz);
        path_pub_->publish(ref_traj);
        RCLCPP_INFO(this->get_logger(), "Published reference trajectory: %zu points", ref_traj.poses.size());

        // 3) 发送 Action Goal
        sendActionGoal(msg->pose);
    }

    finenav_msgs::msg::Trajectory generateStraightLineTrajectory(
        double sx, double sy, double sz,
        double gx, double gy, double gz)
    {
        auto traj = finenav_msgs::msg::Trajectory();
        traj.header.stamp = this->now();
        traj.header.frame_id = map_frame_;
        traj.valid_fields = TRAJ_POSE;
        traj.poses.reserve(num_points_);

        const double dx = gx - sx;
        const double dy = gy - sy;
        const double dz = gz - sz;
        const double yaw = std::atan2(dy, dx);
        const auto q = quatFromYaw(yaw);

        for (int i = 0; i < num_points_; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(num_points_ - 1);
            geometry_msgs::msg::Pose pose;
            pose.position.x = sx + t * dx;
            pose.position.y = sy + t * dy;
            pose.position.z = sz + t * dz;
            pose.orientation = q;
            traj.poses.push_back(pose);
        }

        return traj;
    }

    void sendActionGoal(const geometry_msgs::msg::Pose& goal_pose)
    {
        // 等待 Action 服务器上线
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available, cannot send goal");
            return;
        }

        // 构造 Goal
        auto goal_msg = ComputePlan::Goal();
        goal_msg.planner_id = planner_id_;
        goal_msg.goal = goal_pose;

        RCLCPP_INFO(this->get_logger(),
                    "Sending action goal: planner_id=%s, target=(%.2f, %.2f, %.2f)",
                    planner_id_.c_str(),
                    goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);

        // 设置发送选项
        auto send_goal_options = rclcpp_action::Client<ComputePlan>::SendGoalOptions();

        // 目标响应回调
        send_goal_options.goal_response_callback =
            [this](GoalHandleComputePlan::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
                    return;
                }
                goal_handle_ = goal_handle;
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
            };

        // 结果回调
        send_goal_options.result_callback =
            [this](const GoalHandleComputePlan::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(),
                                    "Plan succeeded! Planning time: %.3f seconds",
                                    rclcpp::Duration(result.result->planning_time).seconds());
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Plan aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(this->get_logger(), "Plan canceled");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d",
                                     static_cast<int>(result.code));
                        break;
                }
                goal_handle_.reset();
            };

        // 异步发送目标
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // ========== 发布者 ==========
    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr path_pub_;      // 参考轨迹
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path_pub_;            // 转换后的 Path（新增）

    // ========== 订阅者 ==========
    rclcpp::Subscription<finenav_msgs::msg::Trajectory>::SharedPtr mppi_plan_sub_;  // MPPI 规划结果（新增）

    // Action 客户端
    rclcpp_action::Client<ComputePlan>::SharedPtr action_client_;
    GoalHandleComputePlan::SharedPtr goal_handle_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 订阅者
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> goal_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>> goal_filter_;

    // 参数
    std::string goal_topic_;
    std::string path_topic_;
    std::string map_frame_;
    std::string base_frame_;
    std::string action_server_name_;
    std::string planner_id_;
    std::string mppi_plan_topic_;     // 新增
    std::string nav_path_topic_;      // 新增
    int num_points_{50};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPoseLinePathTestNode>());
    rclcpp::shutdown();
    return 0;
}