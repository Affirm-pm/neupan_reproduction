// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <any>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "concepts.hpp"

#include "finenav_util/algo_configurator.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_util/trajectory_io/conversions.hpp"
#include "finenav_util/simple_action_server.hpp"

namespace finenav {

// TODO: OneShot v.s. ServoLoop，看看 Nav2 的 PlannerServer 和 ControllerServer 区别

// template <typename... Ts> struct PlannerSet {}; // 可变参数模板
// template <typename... Ts> struct FilterSet {};

template <typename... Ts>
struct PlannerSet {
    using Tuple = std::tuple<Ts...>;
};

// 1. 提取器：给定 PlannerT，提取 std::shared_ptr<PlannerT::IMapView>
template <typename PlannerT>
struct GetMapViewSharedPtr {
    using type = std::shared_ptr<typename PlannerT::IMapView>;
};

// 2. 转换器：Tuple<P...> -> Variant<shared_ptr<P::View>...>
template <typename Tuple> struct ToVariant;

template <typename... Planners>
struct ToVariant<std::tuple<Planners...>> {
    using type = std::variant<typename GetMapViewSharedPtr<Planners>::type...>;
};

// 3. 对外别名
template <typename PlannerSetT>
using GenerateMapViewsVariant = typename ToVariant<typename PlannerSetT::Tuple>::type;


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
    using PlannersT = Traits::Planners; // TODO: 不能有一样的 Planner 类模板，即，每个 Planner 都只能注册一次
    using MapViewsVariant = GenerateMapViewsVariant<PlannersT>;
    using ActionT = finenav_msgs::action::ComputePlan;
    using ActionServer = SimpleActionServer<ActionT>;

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
            action_server_ = std::make_unique<ActionServer>(
                shared_from_this(),
                config_.task_action,
                std::bind(&ControlLayer::execute, this)
            );
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

    virtual MapViewsVariant get_planning_map_view(const std::string& planner_id) = 0; // TODO: 要不要改成回调

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
        // TODO: 接收时，检测 trajectroy_msg is consistent
        // TODO: 这里需要做互斥
        plan_ref_msg_ = msg;
    }

    /// THREAD 1: 执行规划
    using GoalHandleComputePlan = rclcpp_action::ServerGoalHandle<finenav_msgs::action::ComputePlan>;

    void execute() {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto goal = action_server_->get_current_goal();
        auto result = std::make_shared<finenav_msgs::action::ComputePlan::Result>();
        active_planner_id_ = goal->planner_id; // 记录当前请求的规划器 ID

        auto goal_pose = goal->goal;

        // Pose goal_pose_eigen;
        // tf2::fromMsg(goal_msg, goal_pose_eigen);

        // 创建一个周期调用的定时器，定时器频率为控制周期
        {
            // 检查 ROS 客户端请求是否被取消
            if (action_server_->is_cancel_requested()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                action_server_->terminate_all(); // TODO: terminate_all 返回的是 abort 而不是 canceled
                return;
            }


            // 从 TF 获取当前机器人位姿作为起点, map 系下 base_link 的位置，对应 base_link->map 的变换
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero); // TODO: TF 时间戳相关细节，可参考 nav2_planner

            geometry_msgs::msg::Pose start_pose;
            start_pose.position.x = t.transform.translation.x;
            start_pose.position.y = t.transform.translation.y;
            start_pose.position.z = t.transform.translation.z;
            start_pose.orientation = t.transform.rotation;

            // Pose start_pose_eigen = tf2::transformToEigen(t);

            // 在 planners_instance_ 中找匹配 active_planner_id_ 的 planner
            bool planner_found_and_run = false;
            bool plan_success = false;
            finenav_msgs::msg::Trajectory output_traj_msg; // TODO: 后续可能要用unique_ptr封装，否则无法进行进程内 IPC

            auto try_run_planner = [&](auto& planner) -> bool {
                using PlannerT = std::decay_t<decltype(planner)>;
                if (AlgoConfigurator<PlannerT>::name == active_planner_id_) {

                    PlanningContext<PlannerT> ctx; // 填充规划上下文
                    ctx.start_pose = start_pose;
                    ctx.goal_pose = goal_pose;
                    fromROSMsg(std::move(plan_ref_msg_), ctx.ref_traj); // TODO: 这里需要做互斥

                    MapViewsVariant view_variant = this->get_planning_map_view(active_planner_id_);
                    using ExpectedPtrType = std::shared_ptr<typename PlannerT::IMapView>;

                    if (auto* ptr_to_ptr = std::get_if<ExpectedPtrType>(&view_variant)) {
                        auto map_view_ptr = *ptr_to_ptr;
                        if (map_view_ptr) {
                            // TODO: 给地图上锁
                            auto plan_result = planner.plan(ctx, *map_view_ptr);
                            if (plan_result.has_value()) {
                                plan_success = true;
                                toROSMsg(std::move(plan_result.value()), output_traj_msg);
                            }
                            else {
                                RCLCPP_ERROR(this->get_logger(), "User returned a nullptr MapView!");
                            }
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "MapView Type Mismatch for planner %s", active_planner_id_.c_str());
                        }
                    }
                    return true; // 返回 true 以触发短路，停止后续遍历
                }
                return false;
            };

            std::apply([&](auto&... planners) {
                planner_found_and_run = (try_run_planner(planners) || ...);
            }, planners_instance_);

            if (!planner_found_and_run) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Planner with ID " << active_planner_id_ << " not found");
                action_server_->terminate_all();
                return;
            }
            if (!plan_success) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Planner " << active_planner_id_ << " failed to compute a plan");
                action_server_->terminate_all();
                return;
            }

            // 发布规划结果
            // TODO: 发布时， 检测 trajectroy_msg is consistent
            plan_result_pub_->publish(output_traj_msg);


            // 检查是否抵达终点，如果抵达，退出循环 (通过获取系统整体状态，这样的话得再开一个线程)
        }

        // 设置成功结果
        action_server_->succeeded_current();
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }


    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<ActionServer> action_server_;

    rclcpp::Subscription<finenav_msgs::msg::Trajectory>::SharedPtr plan_ref_sub_; // 这里是固定的 Trajectory 消息类型
    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr plan_result_pub_;

    finenav_msgs::msg::Trajectory plan_ref_msg_; // 最近一次收到的规划参考



    // tf buffer 和监听器
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<std::shared_ptr<void>> param_handles_; // 用于存储参数回调句柄，防止被析构


};

}

