// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <any>
#include <mutex>
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
#include "finenav_util/node_thread.hpp"

namespace finenav {

template <typename... Ts>
struct PlannerSet {
    using Tuple = std::tuple<Ts...>;
};

template <typename PlannerT>
struct GetMapViewSharedPtr {
    using type = std::shared_ptr<typename PlannerT::IMapView>;
};

template <typename Tuple> struct ToVariant;

template <typename... Planners>
struct ToVariant<std::tuple<Planners...>> {
    using type = std::variant<typename GetMapViewSharedPtr<Planners>::type...>;
};

// 3. 对外别名
template <typename PlannerSetT>
using GenerateMapViewsVariant = typename ToVariant<typename PlannerSetT::Tuple>::type;

// ---------------------------------------------------------------------------------

struct PrePlanContext {
    const std::string& active_planner_id;       // [RO] 当前请求的规划器 ID
    geometry_msgs::msg::Pose& start_pose;       // [RW] 起始位姿
    geometry_msgs::msg::Pose& goal_pose;        // [RW] 目标位姿
    finenav_msgs::msg::Trajectory& ref_traj;    // [RW] 规划参考轨迹
};

struct PostPlanContext {
    const std::string& active_planner_id;       // [RO] 当前请求的规划器 ID
    const bool is_success;                      // [RO] 规划结果是否成功
    finenav_msgs::msg::Trajectory& result_traj; // [RW] 规划结果轨迹
};

enum class ControlMode {
    OneShot = 0,
    ServoLoop = 1
};

struct ControlLayerConfig { // TODO： 待删除
    std::string task_action;
    std::string input_topic;
    std::string output_topic;
    ControlMode mode; // TODO: OneShot v.s. ServoLoop, 考虑是否需要做成编译期常量，这样允许编译期分支控制流；代价，无法通过参数运行时修改
};

// ==============================================================================
// Definition
// ==============================================================================
template <typename PlannersT>  // TODO: 逐个检查 PlannersT 中的每个规划器是否满足 PlannerConcept; 每个 Planner 只能注册一次
class ControlLayer {
public:
    using MapViewsVariant = GenerateMapViewsVariant<PlannersT>;
    using ActionT = finenav_msgs::action::ComputePlan;
    using ActionServer = SimpleActionServer<ActionT>;

    // 回调签名定义 (修改 PrePlan 返回 bool 以支持短路拦截)
    using MapViewFactory = std::function<MapViewsVariant(const std::string& planner_id)>;
    using PrePlanCallback = std::function<bool(PrePlanContext& cur_ctx)>;
    using PostPlanCallback = std::function<bool(PostPlanContext& cur_ctx)>;

    explicit ControlLayer(rclcpp::Node::SharedPtr node, const std::string& layer_name);

    void setInputTopic(const std::string& topic);
    void setOutputTopic(const std::string& topic);
    std::string getName() const;

    void setupDataPorts(); // TODO: 保护这些 public 接口？

    void set_map_view_factory(MapViewFactory cb) { map_view_factory_ = std::move(cb); }
    void set_on_pre_plan(PrePlanCallback cb) { on_pre_plan_ = std::move(cb); }
    void set_on_post_plan(PostPlanCallback cb) { on_post_plan_ = std::move(cb); }

protected:
    MapViewFactory map_view_factory_;
    PrePlanCallback on_pre_plan_;
    PostPlanCallback on_post_plan_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr pnode_;
    std::unique_ptr<NodeThread> pnode_thread_;

    PlannersT::Tuple planners_instance_;

    std::string active_planner_id_; // 运行时请求的 ID (来自 Action Goal)

    // TODO: onPrePlan(), OnPlanResult() 等函数接口，可能要考虑下语义区分的问题，防止和 LocalLayer 中定义的混淆

    // virtual MapViewsVariant get_planning_map_view(const std::string& planner_id) {}; // TODO: 要不要改成回调

private:
    template <typename T> void keep_alive(std::shared_ptr<T> handle);

    /// THREAD 0: 更新规划参考轨迹，等于主线程？
    void PlanRefCallback(const finenav_msgs::msg::Trajectory& msg);

    /// THREAD 1: 执行规划
    using GoalHandleComputePlan = rclcpp_action::ServerGoalHandle<finenav_msgs::action::ComputePlan>;
    void execute();

private:
    std::string layer_name_;
    std::string input_topic_;
    std::string output_topic_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<ActionServer> action_server_;

    rclcpp::Subscription<finenav_msgs::msg::Trajectory>::SharedPtr plan_ref_sub_;
    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr plan_result_pub_;

    std::mutex plan_ref_mutex_;                  // 新增互斥锁
    finenav_msgs::msg::Trajectory plan_ref_msg_; // 最近一次收到的规划参考

    // tf buffer 和监听器
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<std::shared_ptr<void>> param_handles_; // 用于存储参数回调句柄，防止被析构
};


// ==============================================================================
// ControlLayer 实现部
// ==============================================================================

template <typename PlannersT>
ControlLayer<PlannersT>::ControlLayer(rclcpp::Node::SharedPtr node, const std::string& layer_name)
    : node_(node), layer_name_(layer_name)
{
    pnode_ = rclcpp::Node::make_shared(layer_name, node_->get_name(), rclcpp::NodeOptions()); // TODO：关于 node 参数，后面需要看看该怎么搞，尤其是进程内通信 | layer_name -> layer_name_
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    pnode_->declare_parameter<std::string>("example_param", "hello, world"); // 示例参数

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(pnode_->get_clock()); // TODO: 必须删掉，ControlLayer不负责这个
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    std::string task_action = layer_name;
    if (!task_action.empty()) {    // TODO: 如果 layer_name 是空字符串该怎么办，要不要拦下来
        action_server_ = std::make_unique<ActionServer>(
            pnode_,
            task_action,
            std::bind(&ControlLayer::execute, this)
        );
    }

    // 加载 tuple 中的每个规划器
    std::apply([this](auto&... planners) {
        ([this](auto& p) {
            using T = std::decay_t<decltype(p)>;
            RCLCPP_INFO_STREAM(pnode_->get_logger(), "Loading planner: " << AlgoConfigurator<T>::name);
            keep_alive(AlgoConfigurator<T>::load(pnode_, p));
        }(planners), ...);
    }, planners_instance_);
}

template <typename PlannersT>
void ControlLayer<PlannersT>::setInputTopic(const std::string& topic) {
    input_topic_ = topic;
}

template <typename PlannersT>
void ControlLayer<PlannersT>::setOutputTopic(const std::string& topic) {
    output_topic_ = topic;
}

template <typename PlannersT>
std::string ControlLayer<PlannersT>::getName() const {
    return pnode_->get_name();
}

template <typename PlannersT>
void ControlLayer<PlannersT>::setupDataPorts() {  // TODO: 保护这些 public 接口？
    if (!input_topic_.empty()) {
        plan_ref_sub_ = pnode_->create_subscription<finenav_msgs::msg::Trajectory>(
            input_topic_, 10, std::bind(&ControlLayer::PlanRefCallback, this, std::placeholders::_1));
    }
    if (!output_topic_.empty())  {
        plan_result_pub_ = pnode_->create_publisher<finenav_msgs::msg::Trajectory>(output_topic_, 10);
    }
}

template <typename PlannersT>
template <typename T>
void ControlLayer<PlannersT>::keep_alive(std::shared_ptr<T> handle) {
    if (handle) {
        param_handles_.push_back(handle);
    }
}

template <typename PlannersT>
void ControlLayer<PlannersT>::PlanRefCallback(const finenav_msgs::msg::Trajectory& msg) {
    RCLCPP_INFO(pnode_->get_logger(), "Received plan reference");
    // TODO: 接收时，检测 trajectroy_msg is consistent
    // TODO: 这里需要做互斥
    std::lock_guard<std::mutex> lock(plan_ref_mutex_);
    plan_ref_msg_ = msg;
}

template <typename PlannersT>
void ControlLayer<PlannersT>::execute() {
    RCLCPP_INFO(pnode_->get_logger(), "Executing goal");
    auto goal = action_server_->get_current_goal();
    auto result = std::make_shared<finenav_msgs::action::ComputePlan::Result>();
    active_planner_id_ = goal->planner_id; // 记录当前请求的规划器 ID

    auto goal_pose = goal->goal;

    // 创建一个周期调用的定时器，定时器频率为控制周期
    {
        // 检查 ROS 客户端请求是否被取消
        if (action_server_->is_cancel_requested()) {
            RCLCPP_INFO(pnode_->get_logger(), "Goal canceled");
            action_server_->terminate_all(); // TODO: terminate_all 返回的是 abort 而不是 canceled
            return;
        }

        if (!map_view_factory_) {
            RCLCPP_ERROR(pnode_->get_logger(), "MapViewFactory not set!");
            action_server_->terminate_all(); // TODO: terminate_all 返回的是 abort 而不是 canceled
            return;
        }

        // 从 TF 获取当前机器人位姿作为起点, map 系下 base_link 的位置，对应 base_link->map 的变换
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform( // TODO: 改成从全局读取 current_state
            "map", "base_link", tf2::TimePointZero);

        geometry_msgs::msg::Pose start_pose;

        // 在 planners_instance_ 中找匹配 active_planner_id_ 的 planner
        bool planner_found_and_run = false;
        bool plan_success = false;
        finenav_msgs::msg::Trajectory output_traj_msg; // TODO: 后续可能要用unique_ptr封装，否则无法进行进程内 IPC

        // 安全地拷贝参考轨迹
        finenav_msgs::msg::Trajectory current_ref_msg;
        {
            std::lock_guard<std::mutex> lock(plan_ref_mutex_);
            current_ref_msg = plan_ref_msg_;
        }

        auto try_run_planner = [&](auto& planner) -> bool {
            using PlannerT = std::decay_t<decltype(planner)>;
            if (AlgoConfigurator<PlannerT>::name == active_planner_id_) {

                auto opt_ctx = [&]() -> std::optional<PlanningContext<PlannerT>> {
                    PrePlanContext pre_ctx {
                        active_planner_id_,
                        start_pose,
                        goal_pose,
                        current_ref_msg,
                    };

                    if (on_pre_plan_ && !on_pre_plan_(pre_ctx)) {
                        // 如果用户拦截，返回空壳，宣告构建中止
                        return std::nullopt;
                    }

                    PlanningContext<PlannerT> ctx;
                    ctx.start_pose = pre_ctx.start_pose;
                    ctx.goal_pose = pre_ctx.goal_pose;
                    fromROSMsg(std::move(pre_ctx.ref_traj), ctx.ref_traj); // TODO: 这里需要做互斥
                    return ctx;
                }();

                if (!opt_ctx.has_value()) {
                    return true; // 返回 true 以触发短路，停止后续遍历
                }
                auto& ctx = opt_ctx.value();

                auto view_variant = map_view_factory_(active_planner_id_);
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
                            RCLCPP_ERROR(pnode_->get_logger(), "User returned a nullptr MapView!");
                        }
                    } else {
                        RCLCPP_ERROR(pnode_->get_logger(), "MapView Type Mismatch for planner %s", active_planner_id_.c_str());
                    }
                }

                if (on_post_plan_) {
                    PostPlanContext post_ctx {
                        active_planner_id_,
                        plan_success,      // 把底层的真实结果告诉用户
                        output_traj_msg,
                    };

                    plan_success = on_post_plan_(post_ctx);
                }
                return true; // 返回 true 以触发短路，停止后续遍历
            }
            return false;
        };

        std::apply([&](auto&... planners) {
            planner_found_and_run = (try_run_planner(planners) || ...);
        }, planners_instance_);

        if (!planner_found_and_run) {
            RCLCPP_ERROR_STREAM(pnode_->get_logger(), "Planner with ID " << active_planner_id_ << " not found");
            action_server_->terminate_all();
            return;
        }
        if (!plan_success) {
            RCLCPP_ERROR_STREAM(pnode_->get_logger(), "Planner " << active_planner_id_ << " failed to compute a plan");
            action_server_->terminate_all();
            return;
        }

        // 发布规划结果
        // TODO: 发布时， 检测 trajectroy_msg is consistent
        plan_result_pub_->publish(output_traj_msg);

        // 通过current_state,检查是否抵达终点，如果抵达
    }

    // 设置成功结果
    action_server_->succeeded_current();
    RCLCPP_INFO(pnode_->get_logger(), "Goal succeeded");
}

} // namespace finenav