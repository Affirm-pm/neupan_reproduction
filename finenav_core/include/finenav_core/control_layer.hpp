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

#include "finenav_util/algo_configurator.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_msgs/action/compute_plan.hpp"

namespace finenav {

// TODO: OneShot v.s. ServoLoop，看看 Nav2 的 PlannerServer 和 ControllerServer 区别

// template <typename... Ts> struct PlannerSet {}; // 可变参数模板
// template <typename... Ts> struct FilterSet {};

template <typename... Ts>
struct PlannerSet {
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
requires MapConcept<typename Traits::Map>   // TODO: 逐个检查 PlannersT 中的每个规划器是否满足 PlannerConcept
class ControlLayer : public rclcpp::Node { // TODO: 改名 AbstracControltLayer 或者 ControlLayerBase
  public:
    using MapT      = Traits::Map;
    using PlannersT = Traits::Planners;
    using MapViewsT = Traits::MapViews;

    explicit ControlLayer(const std::string& name, const rclcpp::NodeOptions& options, const ControlLayerConfig config)
        : Node(name, options), config_(config) {



        // --- 2. 初始化 ROS 基础设施 ---
        // 从 ROS2 参数服务器加载配置
        this->declare_parameter<std::string>("example_param", "hello, world"); // 示例参数


        // --- 3. 初始化成员变量 ---
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

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
            plan_ref_sub_ = this->template create_subscription<finenav_msgs::msg::Trajectory>(
                config_.input_topic, 10, std::bind(&ControlLayer::PlanRefCallback, this, std::placeholders::_1));
        }
        if (!config_.output_topic.empty()) {
            plan_result_pub_ = this->create_publisher<finenav_msgs::msg::Trajectory>(config_.output_topic, 10);
        }
    }

    void initialize() {

        keep_alive(AlgoConfigurator<MapT>::load(this->shared_from_this(), map_instance_));
        std::cout << "Create Map Instance: " << AlgoConfigurator<MapT>::name << std::endl;
        std::cout << "size of param_handles: " << param_handles_.size() << std::endl;

        // 配置加载 tuple 中的每个规划器
        std::apply([this](auto&... planners) {
            ([this](auto& p) {
                using T = std::decay_t<decltype(p)>;
                RCLCPP_INFO_STREAM(this->get_logger(), "Loading planner: " << AlgoConfigurator<T>::name);
                keep_alive(AlgoConfigurator<T>::load(this->shared_from_this(), p));
            }(planners), ...);
        }, planners_instance_);

    }


  protected:
    ControlLayerConfig config_;

    MapT map_instance_;
    PlannersT::Tuple planners_instance_;

    std::string active_planner_id_; // 运行时请求的 ID (来自 Action Goal)

    // TODO: onPrePlan(), OnPlanResult() 等函数接口，可能要考虑下语义区分的问题，防止和 LocalLayer 中定义的混淆
    virtual MapViewsT get_planning_map_view() = 0; // TODO: 要不要改成回调

  private:

    template <typename T>
    void keep_alive(std::shared_ptr<T> handle) {
        if (handle) {
            param_handles_.push_back(handle);
        }
    }

    /// THREAD 0: 更新规划参考轨迹，等于主线程？
    void PlanRefCallback(const finenav_msgs::msg::Trajectory& msg) {
        RCLCPP_INFO(this->get_logger(), "Received plan reference");
        // TODO: 接收时，assert trajectroy_msg is consistent
        // TODO: 这里需要做互斥
        plan_ref_msg_ = msg;


    }

    /// THREAD 1: 执行规划
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
        active_planner_id_ = goal->planner_id; // 记录当前请求的规划器 ID

        // 在 planners_instance_ 中找匹配 active_planner_id_ 的 planner

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



            // 找到这个规划器
            finenav_msgs::msg::Trajectory output_traj;
            bool planner_found_and_run = false;

            auto try_run_planner = [&](auto& planner) -> bool {
                using PlannerT = std::decay_t<decltype(planner)>;

                PlanningContext<PlannerT> ctx; // 填充规划上下文
                if (AlgoConfigurator<PlannerT>::name == active_planner_id_) {

                    // TODO: 给地图上锁
                    // ctx.reference_path = plan_ref_msg_;
                    auto plan_result = planner.plan(ctx, this->get_planning_map_view());
                    return true; // 返回 true 以触发短路，停止后续遍历
                }
                return false;
            };
            std::apply([&](auto&... planners) {
                planner_found_and_run = (try_run_planner(planners) || ...);
            }, planners_instance_);

            if (!planner_found_and_run) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Planner with ID " << active_planner_id_ << " not found");
                goal_handle->abort(result);
                return;
            }
            // if (!plan_success) {
            //     RCLCPP_ERROR_STREAM(this->get_logger(), "Planner " << active_planner_id_ << " failed to compute a plan");
            //     goal_handle->abort(result);
            //     return;
            // }

            // 发布规划结果
            plan_result_pub_->publish(output_traj);
            // TODO: 发布时， assert trajectroy_msg is consistent

            // 检查是否抵达终点，如果抵达，退出循环 (通过获取系统整体状态)
        }

        // 设置成功结果
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }


    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_action::Server<finenav_msgs::action::ComputePlan>::SharedPtr action_server_;



    rclcpp::Subscription<finenav_msgs::msg::Trajectory>::SharedPtr plan_ref_sub_; // 这里是固定的 Trajectory 消息类型
    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr plan_result_pub_;

    finenav_msgs::msg::Trajectory plan_ref_msg_; // 最近一次收到的规划参考



    // tf buffer 和监听器
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<std::shared_ptr<void>> param_handles_; // 用于存储参数回调句柄，防止被析构


};

};

