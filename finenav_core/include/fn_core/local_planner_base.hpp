// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// #include <nav2_msgs/action/compute_path_to_pose.hpp> // TODO: 自定义 Action
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "concepts.hpp"
#include "finenav_interfaces/msg/map_chunk.hpp"
#include "finenav_interfaces/msg/plan_result.hpp"
#include "finenav_interfaces/action/compute_plan.hpp"

namespace finenav {

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

template <LayerTraits Traits>
class LocalPlannerBase : public rclcpp::Node {
  public:
    using MapT      = Traits::Map;
    using PlannersT = Traits::Planners;
    using FiltersT  = Traits::Filters; // TODO: 需要能支持空的

    // Action
    // TODO: 需要自己定义 Action 交互接口
    // using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    // using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;

    explicit LocalPlannerBase(const rclcpp::NodeOptions & options) : Node("local_planner_base", options) {
        /// --- 1. 静态安检 (Concept Check) ---
        // Concept 本质上就是一个接受类型作为参数，并返回布尔值（true/false）的模板函数/变量。
        static_assert(MapConcept<MapT>, "MapT does not satisfy MapConcept");
        // TODO: 逐个检查 PlannersT 中的每个规划器是否满足 PlannerConcept, 必须用递归模板展开的方式来实现对可变参数模板的检查


        // --- 2. 初始化 ROS 基础设施 ---
        // 从 ROS2 参数服务器加载配置
        //
        this->declare_parameter<std::string>("example_param", "hello, world"); // 示例参数


        // --- 2.5. 初始化成员变量 ---
        // 地图实例化
        map_instance_.configure();

        // C++17 std::apply 用于将元组中的元素展开并传递给一个可调用对象（如函数或 lambda 表达式）
        // std::apply(Function, Tuple);
        // 等价于 Function(TupleElement1, TupleElement2, ..., TupleElementN);
        std::apply([this](auto&... planners) {
            // 对每一个 planner p，调用 p.configure(...)
            (planners.configure(this, "planners." + std::string(planners.name)), ...); // 逗号折叠，这里的逗号表达式确保了顺序执行
        }, planners_instance_);


        // --- 3. 控制流 ---
        // 基类负责所有控制流的相关处理：包括 handle_goal, handle_cancel, handle_accepted
        // action_server_ = rclcpp_action::create_server<ComputePathToPose>(
        //     this,
        //     "compute_path_to_pose",
        //     std::bind(&LocalPlannerBase::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        //     std::bind(&LocalPlannerBase::handle_cancel, this, std::placeholders::_1),
        //     std::bind(&LocalPlannerBase::handle_accepted, this, std::placeholders::_1));


        // --- 4. 地图数据流 ---
        // 基类负责所有数据流的相关处理：
        // (1) 订阅点云回调，将 pointcloud2_msg 转换为 FineNav 内部类型，然后调用 map_.update()
        // (2) 检测机器人定位，一旦定位变化，调用视窗移动函数，将过时数据通过回调函数发布出去；向 L3 请求加载新数据
        // TODO: 需要单独开一个线程，持续监测 TF ，如果有其他的 ROS基础信息，也可以持续监测；tf_buffer_是公用资源，需要考虑线程安全

        // 在模板类中调用依赖于模板参数的成员函数时，编译器无法确定 create_subscription  是普通函数还是模板函数
        pointcloud_sub_ = this->template create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10,  [this](const sensor_msgs::msg::PointCloud2 & msg) {
                RCLCPP_INFO(this->get_logger(), "Subscribing to points");
            });

        // B. 地图交互端口 (L2 <-> L3)
        // TODO: offload_pub_ 和 load_sub_ 都需要一个对应的 service 向服务器请求，只有服务器回复 yes，才用 topic 进行数据传输
        offload_pub_ = this->create_publisher<fn_interfaces::msg::MapChunk>("map_offload", 10);
        // TODO: 该怎么知道应该如何用手上的map装填MapChunk呢？

        load_sub_ = this->template create_subscription<fn_interfaces::msg::MapChunk>(
            "map_load", 10,
            [this](const fn_interfaces::msg::MapChunk::SharedPtr msg) {
                // server 发过来了 MapChunk，其中包含了 (x, y, z, filed1, field2, ...)
                // 基类不知道该如何根据这些东西具体更新地图，需要将其交给用户
                // impl().map_.update(msg);
                RCLCPP_INFO(this->get_logger(), "Received map chunk to load");
            });

        // --- 5. 规划数据流 ---
        // A. PlanRef 的订阅。不一定需要订阅，需要由创建的 Planner 类控制，用 constexpr 编译流分支即可
        // 情况1：planner 宣布自己不需要 ref，则不创建该订阅
        // 情况2：planner 宣布自己需要 ref，并且指明具体需要哪些 ref，则创建该订阅，缓存最新的 ref_msg；基类同时需要负责在数据到来时，将 ref_msg 变为结构化数据 PlanningContext
        // if constexpr ....
        // plan_ref_sub_ = this->template create_subscription<fn_interfaces::msg::PlanResult>(
        //     "plan_ref", 10,
        //     [this](const nav_msgs::msg::Path::SharedPtr msg) {
        //         RCLCPP_INFO(this->get_logger(), "Received plan reference");
        //     });

        // B. PlanResult 的发布
        plan_result_pub_ = this->create_publisher<fn_interfaces::msg::PlanResult>("plan_result", 10);


    }

  protected:
    MapT map_instance_;
    PlannersT::Tuple planners_instance_;
    FiltersT::Tuple filters_instance_;

    std::string active_planner_id_; // 运行时请求的 ID (来自 Action Goal)



  private:
    // rclcpp_action::GoalResponse handle_goal(
    //     const rclcpp_action::GoalUUID & uuid,
    //     std::shared_ptr<const ComputePathToPose::Goal> goal) {
    //     RCLCPP_INFO(this->get_logger(), "Received goal request");
    //     (void)uuid;
    //     (void)goal;
    //     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    // }
    //
    // rclcpp_action::CancelResponse handle_cancel(
    //     const std::shared_ptr<GoalHandleComputePathToPose> goal_handle) {
    //     RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    //     (void)goal_handle;
    //     return rclcpp_action::CancelResponse::ACCEPT;
    // }
    //
    // void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle) {
    //
    //     // TODO: 为什么这里需要新开一个线程？ROS2官方例程是怎么做的，Nav2是怎么做的？
    //     std::thread{std::bind(&LocalPlannerBase::execute, this, std::placeholders::_1), goal_handle}.detach();
    // }
    //
    // void execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle) {
    //     RCLCPP_INFO(this->get_logger(), "Executing goal");
    //     const auto goal = goal_handle->get_goal();
    //     auto result = std::make_shared<ComputePathToPose::Result>();
    //
    //     // TODO: 从 TF 获取当前机器人位姿作为起点
    //     Pose start_pose;
    //
    //     // 从 goal 消息中获取终点位姿
    //     Pose goal_pose;
    //     // TODO: 四元数转欧拉角
    //
    //     // 调用CRTP派生类中的规划器进行规划
    //     auto planned_path = impl().planner_.plan(start_pose, goal_pose, impl().map_);
    //
    //     // 将规划结果打包到 action result 中
    //     // TODO: 转换 planner_.plan 的输出到 nav_msgs::msg::Path
    //     // result->path = planned_path;
    //
    //     // TODO: 这种工程上的复杂逻辑可以直接看 Nav2，没必要自己死磕
    //     // 检查 ROS 客户端请求是否被取消
    //     if (goal_handle->is_canceling()) {
    //         goal_handle->canceled(result);
    //         RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //         return;
    //     }
    //
    //     // 设置成功结果
    //     goal_handle->succeed(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    // }

    // --- 成员变量 ---
    rclcpp::TimerBase::SharedPtr timer_;

    // rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    rclcpp::Subscription<finenav_interfaces::msg::PlanResult>::SharedPtr plan_ref_sub_; // 这里是固定的 Trajectory 消息类型
    rclcpp::Publisher<finenav_interfaces::msg::PlanResult>::SharedPtr plan_result_pub_;

    // 地图交互
    rclcpp::Publisher<finenav_interfaces::msg::MapChunk>::SharedPtr offload_pub_;
    rclcpp::Subscription<finenav_interfaces::msg::MapChunk>::SharedPtr load_sub_;


};

};

