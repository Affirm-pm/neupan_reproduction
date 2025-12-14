// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>



#include "concepts.hpp"

#include "finenav_msgs/msg/plan_result.hpp"
#include "finenav_msgs/action/compute_plan.hpp"

namespace finenav {

// TODO: OneShot v.s. ServoLoop，看看 Nav2 的 PlannerServer 和 ControllerServer 区别

// template <typename... Ts> struct PlannerSet {}; // 可变参数模板
// template <typename... Ts> struct FilterSet {};

template <typename... Ts>
struct PlannerSet {
    using Tuple = std::tuple<Ts...>;
};

template <typename... Ts>
struct FilterSet {
    using Tuple = std::tuple<Ts...>;
};

enum class ControlMode {
    OneShot = 0,
    ServoLoop = 1
};

struct ControlLayerConfig {
    std::string task_action;
    std::string input_topic;
    std::string output_topic;
    ControlMode mode; // TODO: 考虑是否需要做成编译期常量，这样允许编译期分支控制流；代价，无法通过参数运行时修改
};

template <LayerTraits Traits>
class ControlLayer : public rclcpp::Node {
  public:
    using MapT      = Traits::Map;
    using PlannersT = Traits::Planners;
    using FiltersT  = Traits::Filters; // TODO: 需要能支持空的


    explicit ControlLayer(const std::string& name, const rclcpp::NodeOptions& options, const ControlLayerConfig config)
        : Node(name, options), config_(config) {
        /// --- 1. 静态安检 (Concept Check) ---
        // Concept 本质上就是一个接受类型作为参数，并返回布尔值（true/false）的模板函数/变量。
        static_assert(MapConcept<MapT>, "MapT does not satisfy MapConcept");
        // TODO: 逐个检查 PlannersT 中的每个规划器是否满足 PlannerConcept, 必须用递归模板展开的方式来实现对可变参数模板的检查


        // --- 2. 初始化 ROS 基础设施 ---
        // 从 ROS2 参数服务器加载配置
        this->declare_parameter<std::string>("example_param", "hello, world"); // 示例参数


        // --- 3. 初始化成员变量 ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // 地图实例化
        map_instance_.configure();

        // 规划器实例化
        // C++17 std::apply 用于将元组中的元素展开并传递给一个可调用对象（如函数或 lambda 表达式）
        // std::apply(Function, Tuple);
        // 等价于 Function(TupleElement1, TupleElement2, ..., TupleElementN);
        std::apply([this](auto&... planners) {
            // 对每一个 planner p，调用 p.configure(...)
            (planners.configure(this, "planners." + std::string(planners.name)), ...); // 逗号折叠，这里的逗号表达式确保了顺序执行
        }, planners_instance_);


        // --- 4. 控制流 ---
        // TODO: 看看 Nav2 如何封装 ActionServer 逻辑，拿来主义
        if (!config_.task_action.empty()) {
            action_server_ = rclcpp_action::create_server<finenav_msgs::action::ComputePlan>(
                this,
                config_.task_action,
                std::bind(&ControlLayer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ControlLayer::handle_cancel, this, std::placeholders::_1),
                std::bind(&ControlLayer::handle_accepted, this, std::placeholders::_1));
        }

        // --- 5. 规划数据流 ---
        if (!config_.input_topic.empty()) {
            plan_ref_sub_ = this->template create_subscription<finenav_msgs::msg::PlanResult>(
                config_.input_topic, 10,
                [this](const finenav_msgs::msg::PlanResult::SharedPtr msg) {
                    RCLCPP_INFO(this->get_logger(), "Received plan reference");
                });
        }
        if (!config_.output_topic.empty()) {
            plan_result_pub_ = this->create_publisher<finenav_msgs::msg::PlanResult>(config_.output_topic, 10);
        }

    }


  protected:
    ControlLayerConfig config_;

    MapT map_instance_;
    PlannersT::Tuple planners_instance_;
    FiltersT::Tuple filters_instance_;

    std::string active_planner_id_; // 运行时请求的 ID (来自 Action Goal)


  private:
    using GoalHandleComputePlan = rclcpp_action::ServerGoalHandle<finenav_msgs::action::ComputePlan>;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const finenav_msgs::action::ComputePlan::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleComputePlan> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accepted(
        const std::shared_ptr<GoalHandleComputePlan> goal_handle) {
        std::thread{std::bind(&ControlLayer::execute, this, std::placeholders::_1), goal_handle}.detach();
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
    }

    void execute(const std::shared_ptr<GoalHandleComputePlan> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<finenav_msgs::action::ComputePlan::Result>();

        // 从 goal 消息中获取终点位姿
        Pose goal_pose;
        auto goal_msg = goal->goal;
        tf2::fromMsg(goal_msg, goal_pose);

        // 创建一个周期调用的定时器，定时器频率为控制周期
        {
            // 检查 ROS 客户端请求是否被取消
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            /****** 进行一次规划 ******/
            // 从 TF 获取当前机器人位姿作为起点, map 系下 base_link 的位置，对应 base_link->map 的变换
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero); // TODO: TF 时间戳相关细节，可参考 nav2_planner
            Pose start_pose = tf2::transformToEigen(t);

            // 调用 planner, 用 constexpr 编译流分支来区分规划器是否需要 ref
            // 如果规划失败，报错，退出

            // 进行 filter 处理

            // 发布规划结果

            // 检查是否抵达终点，如果抵达，退出循环
        }

        // 设置成功结果
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }


    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_action::Server<finenav_msgs::action::ComputePlan>::SharedPtr action_server_;



    rclcpp::Subscription<finenav_msgs::msg::PlanResult>::SharedPtr plan_ref_sub_; // 这里是固定的 Trajectory 消息类型
    rclcpp::Publisher<finenav_msgs::msg::PlanResult>::SharedPtr plan_result_pub_;



    // tf buffer 和监听器
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


};

};

