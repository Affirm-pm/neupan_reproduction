// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <any>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_msgs/msg/robot_state.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_core/plan/conversions.hpp"
#include "finenav_core/concepts.hpp"
#include "finenav_core/map/map_server.hpp"
#include "finenav_util/algo_configurator.hpp"
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
    const std::string& active_planner_id;            // [RO] 当前请求的规划器 ID
    finenav_msgs::msg::RobotState& robot_state;      // [RW] 当前机器人状态（可被回调修改）
    geometry_msgs::msg::Pose& goal_pose;             // [RW] 目标位姿
    finenav_msgs::msg::Trajectory& ref_traj;         // [RW] 规划参考轨迹
};

struct PostPlanContext {
    const std::string& active_planner_id;       // [RO] 当前请求的规划器 ID
    const bool is_success;                      // [RO] 规划结果是否成功
    finenav_msgs::msg::Trajectory& result_traj; // [RW] 规划结果轨迹
};

// ---------------------------------------------------------------------------------
// Execution Policies
// ---------------------------------------------------------------------------------

/**
 * @brief SingleShotPolicy — 每次 Action Goal 执行一次 plan()，完成后立即 succeed。
 *
 * 适用于离线规划、测试等"请求-响应"场景。
 * planner.reset() 在 execute() 开始时调用（Episode 边界）。
 */
struct SingleShotPolicy {};

/**
 * @brief TrackingPolicy — 每次 Action Goal 以固定频率循环调用 plan()，直到取消或目标到达。
 *
 * 适用于 MPPI 等需要持续追踪的在线控制器。
 * planner.reset() 仅在 execute() 开始时调用一次；循环内各帧 plan() 共享热启动状态。
 *
 * @param frequency_hz  控制循环频率（Hz），默认 10 Hz。
 */
struct TrackingPolicy {
    double frequency_hz = 10.0;
};

// ==============================================================================
// ControlLayer Definition
// ==============================================================================

/**
 * @brief ControlLayer — 将 Planner、MapView 与执行策略绑定为一个规划层。
 *
 * @tparam PlannersT      PlannerSet<...>，枚举所有支持的规划器类型。
 * @tparam ExecutionPolicy  执行策略类型，默认 SingleShotPolicy。
 *                          支持扩展：新增策略只需定义 policy struct + 添加对应 executeImpl() 重载。
 */
template <typename PlannersT, typename ExecutionPolicy = SingleShotPolicy>
class ControlLayer {
public:
    using MapViewsVariant = GenerateMapViewsVariant<PlannersT>;
    using ActionT = finenav_msgs::action::ComputePlan;
    using ActionServer = SimpleActionServer<ActionT>;

    // 框架内部用于携带地图视图及其对应的 RAII 读锁的聚合体。
    // lock_guard 持有 LockedMapRO<MapT> 的 shared_ptr，保证在 planner.plan() 整个
    // 调用期间读锁不被释放；view_variant 是规划器实际读取的视图指针。
    struct BoundMapView {
        MapViewsVariant view_variant;
        std::shared_ptr<void> lock_guard; // 持有 LockedMapRO<MapT>，离开作用域后自动释放读锁
    };

    using MapViewFactory        = std::function<BoundMapView()>;
    using PrePlanCallback       = std::function<bool(PrePlanContext&)>;
    using PostPlanCallback      = std::function<bool(PostPlanContext&)>;
    using RobotStateProvider    = std::function<finenav_msgs::msg::RobotState()>;

    explicit ControlLayer(
        rclcpp::Node::SharedPtr node,
        const std::string& layer_name,
        ExecutionPolicy policy = ExecutionPolicy{});

    void setInputTopic(const std::string& topic);
    void setOutputTopic(const std::string& topic);
    std::string getName() const;

    void setupDataPorts(); // TODO: 保护这些 public 接口？

    /**
     * @brief 将 MapServer 与规划器所需的地图视图绑定。
     *
     * 框架在每次调用 planner.plan() 前自动通过 server 申请读锁，并将加锁后的
     * const MapT& 传给 builder 以构造 IMapView；读锁在 plan() 返回后自动释放。
     * 这确保规划期间地图数据不被写入线程修改，同时对用户完全透明。
     *
     * BuilderFn 的签名要求：接受 const MapT& 参数，返回值可构造 MapViewsVariant
     * （即返回 variant 内任意一种 shared_ptr 子类型即可，无需显式转换）。
     *
     * @tparam MapT      地图类型，从 server 参数自动推导，必须满足 MapConcept
     * @tparam BuilderFn 任意可调用类型（lambda / 函数对象等），签名受 requires 约束
     * @param server     提供地图数据的 MapServer，通过弱引用持有（不延长生命周期）
     * @param builder    用户提供的适配函数：接收加锁的 const MapT&，返回可构造 MapViewsVariant 的值
     */
    template <MapConcept MapT, typename BuilderFn>
    requires std::is_invocable_v<BuilderFn, const MapT&> &&
             std::convertible_to<std::invoke_result_t<BuilderFn, const MapT&>, MapViewsVariant>
    void bind_map_view(std::shared_ptr<MapServer<MapT>> server, BuilderFn&& builder);

    void set_on_pre_plan(PrePlanCallback cb) { on_pre_plan_ = std::move(cb); }
    void set_on_post_plan(PostPlanCallback cb) { on_post_plan_ = std::move(cb); }

    /**
     * @brief 注册机器人状态提供者。
     *
     * 框架在每次 planOnce() 开始时调用此回调获取当前 RobotState，
     * 取代原来基于 TF 查询的 getCurrentPose()。
     * 典型实现：从 FineNavEngine Localizer 读取最新 NavState 并转换为 RobotState。
     */
    void set_robot_state_provider(RobotStateProvider provider) {
        robot_state_provider_ = std::move(provider);
    }

    const std::string& get_active_planner_id() const { return active_planner_id_; }

protected:
    MapViewFactory map_view_factory_;
    PrePlanCallback on_pre_plan_;
    PostPlanCallback on_post_plan_;
    RobotStateProvider robot_state_provider_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr pnode_;
    std::unique_ptr<NodeThread> pnode_thread_;

    typename PlannersT::Tuple planners_instance_;

    std::string active_planner_id_;
    ExecutionPolicy policy_;

    // TODO: onPrePlan(), OnPlanResult() 等函数接口，可能要考虑下语义区分的问题，防止和 LocalLayer 中定义的混淆

    // virtual MapViewsVariant get_planning_map_view(const std::string& planner_id) {}; // TODO: 要不要改成回调

private:
    template <typename T> void keep_alive(std::shared_ptr<T> handle);

    void PlanRefCallback(const finenav_msgs::msg::Trajectory& msg);

    /// THREAD 1: 执行规划 — Episode 入口，处理边界后派发给 executeImpl()
    using GoalHandleComputePlan = rclcpp_action::ServerGoalHandle<finenav_msgs::action::ComputePlan>;
    void execute();

    // ------------------------------------------------------------------
    // Policy dispatch — 新增执行模式：定义 policy struct + 添加此处重载
    // ------------------------------------------------------------------
    void executeImpl(SingleShotPolicy& policy);
    void executeImpl(TrackingPolicy& policy);

    // ------------------------------------------------------------------
    // 共享 Helpers（两种策略均使用）
    // ------------------------------------------------------------------

    /** @brief 对匹配 active_planner_id_ 的规划器调用 reset()，在 Episode 边界处触发。 */
    void resetActivePlanner();

    /**
     * @brief 执行一次完整规划步骤：pre-cb → ctx → 地图锁 → plan() → post-cb → 发布。
     * @return true 表示规划成功并发布了结果。
     */
    bool planOnce(geometry_msgs::msg::Pose goal_pose);


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


    std::vector<std::shared_ptr<void>> param_handles_; // 用于存储参数回调句柄，防止被析构
};


// ==============================================================================
// ControlLayer 实现部
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
ControlLayer<PlannersT, ExecutionPolicy>::ControlLayer(
    rclcpp::Node::SharedPtr node,
    const std::string& layer_name,
    ExecutionPolicy policy)
    : node_(node), layer_name_(layer_name), policy_(std::move(policy))
{
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(layer_name, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    pnode_->declare_parameter<std::string>("example_param", "hello, world");


    if (!layer_name.empty()) {
        action_server_ = std::make_unique<ActionServer>(
            pnode_,
            layer_name,
            std::bind(&ControlLayer::execute, this));
    }

    std::apply([this](auto&... planners) {
        ([this](auto& p) {
            using T = std::decay_t<decltype(p)>;
            RCLCPP_INFO_STREAM(pnode_->get_logger(), "Loading planner: " << AlgoConfigurator<T>::name);
            keep_alive(AlgoConfigurator<T>::load(pnode_, p));
        }(planners), ...);
    }, planners_instance_);
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::setInputTopic(const std::string& topic) {
    input_topic_ = topic;
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::setOutputTopic(const std::string& topic) {
    output_topic_ = topic;
}

template <typename PlannersT, typename ExecutionPolicy>
std::string ControlLayer<PlannersT, ExecutionPolicy>::getName() const {
    return pnode_->get_name();
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::setupDataPorts() {
    if (!input_topic_.empty()) {
        plan_ref_sub_ = pnode_->create_subscription<finenav_msgs::msg::Trajectory>(
            input_topic_, 10,
            std::bind(&ControlLayer::PlanRefCallback, this, std::placeholders::_1));
    }
    if (!output_topic_.empty()) {
        plan_result_pub_ = pnode_->create_publisher<finenav_msgs::msg::Trajectory>(output_topic_, 10);
    }
}

template <typename PlannersT, typename ExecutionPolicy>
template <typename T>
void ControlLayer<PlannersT, ExecutionPolicy>::keep_alive(std::shared_ptr<T> handle) {
    if (handle) {
        param_handles_.push_back(handle);
    }
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::PlanRefCallback(
    const finenav_msgs::msg::Trajectory& msg)
{
    std::lock_guard<std::mutex> lock(plan_ref_mutex_);
    plan_ref_msg_ = msg;
}

// ------------------------------------------------------------------------------
// 共享 Helpers
// ------------------------------------------------------------------------------

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::resetActivePlanner() {
    std::apply([&](auto&... planners) {
        ([&](auto& p) {
            using PlannerT = std::decay_t<decltype(p)>;
            if (AlgoConfigurator<PlannerT>::name == active_planner_id_) {
                if constexpr (requires { p.reset(); }) {
                    p.reset();
                    RCLCPP_INFO(pnode_->get_logger(),
                        "Planner '%s' reset at episode boundary.", active_planner_id_.c_str());
                }
            }
        }(planners), ...);
    }, planners_instance_);
}

template <typename PlannersT, typename ExecutionPolicy>
bool ControlLayer<PlannersT, ExecutionPolicy>::planOnce(geometry_msgs::msg::Pose goal_pose) {
    if (!map_view_factory_) {
        RCLCPP_ERROR(pnode_->get_logger(), "MapViewFactory not set!");
        return false;
    }

    if (!robot_state_provider_) {
        RCLCPP_ERROR(pnode_->get_logger(), "RobotStateProvider not set! Call set_robot_state_provider() first.");
        return false;
    }
    finenav_msgs::msg::RobotState current_robot_state = robot_state_provider_();

    finenav_msgs::msg::Trajectory current_ref_msg;
    {
        std::lock_guard<std::mutex> lock(plan_ref_mutex_);
        current_ref_msg = plan_ref_msg_;
    }

    bool plan_success  = false;
    bool planner_found = false;
    finenav_msgs::msg::Trajectory output_traj_msg;

    auto try_run_planner = [&](auto& planner) -> bool {
        using PlannerT = std::decay_t<decltype(planner)>;
        if (AlgoConfigurator<PlannerT>::name != active_planner_id_) {
            return false;
        }

        auto opt_ctx = [&]() -> std::optional<PlanningContext<PlannerT>> {
            PrePlanContext pre_ctx{
                active_planner_id_, current_robot_state, goal_pose, current_ref_msg};

            if (on_pre_plan_ && !on_pre_plan_(pre_ctx)) {
                return std::nullopt; // 用户拦截，中止此次规划
            }

            PlanningContext<PlannerT> ctx;
            ctx.robot_state = pre_ctx.robot_state;
            ctx.goal_pose   = pre_ctx.goal_pose;
            fromROSMsg(std::move(pre_ctx.ref_traj), ctx.ref_traj);
            return ctx;
        }();

        if (!opt_ctx.has_value()) {
            return true; // 短路遍历
        }

        auto& ctx   = opt_ctx.value();
        auto  bound = map_view_factory_();

        using ExpectedPtrType = std::shared_ptr<typename PlannerT::IMapView>;
        if (auto* ptr = std::get_if<ExpectedPtrType>(&bound.view_variant)) {
            if (*ptr) {
                // bound.lock_guard 持有读锁，plan() 调用期间地图不被写入
                auto result = planner.plan(ctx, **ptr);
                if (result.has_value()) {
                    plan_success = true;
                    toROSMsg(std::move(result.value()), output_traj_msg);
                } else {
                    RCLCPP_WARN(pnode_->get_logger(), "plan() returned nullopt.");
                }
            } else {
                RCLCPP_ERROR(pnode_->get_logger(),
                    "MapView ptr is null for planner '%s'.", active_planner_id_.c_str());
            }
        } else {
            RCLCPP_ERROR(pnode_->get_logger(),
                "MapView type mismatch for planner '%s'.", active_planner_id_.c_str());
        }

        if (on_post_plan_) {
            PostPlanContext post_ctx{active_planner_id_, plan_success, output_traj_msg};
            plan_success = on_post_plan_(post_ctx);
        }

        return true; // 短路遍历
    };

    std::apply([&](auto&... planners) {
        planner_found = (try_run_planner(planners) || ...);
    }, planners_instance_);

    if (!planner_found) {
        RCLCPP_ERROR_STREAM(pnode_->get_logger(),
            "Planner '" << active_planner_id_ << "' not found.");
        return false;
    }

    if (plan_success && plan_result_pub_) {
        plan_result_pub_->publish(output_traj_msg);
    }

    return plan_success;
}

// ------------------------------------------------------------------------------
// execute() — Episode 入口
// ------------------------------------------------------------------------------

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::execute() {
    RCLCPP_INFO(pnode_->get_logger(), "New goal received — starting episode.");

    auto goal          = action_server_->get_current_goal();
    active_planner_id_ = goal->planner_id;

    // Episode 边界：在任何 plan() 调用前重置规划器内部状态
    resetActivePlanner();

    // 按策略执行
    executeImpl(policy_);
}

// ------------------------------------------------------------------------------
// executeImpl — SingleShotPolicy
// ------------------------------------------------------------------------------

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::executeImpl(SingleShotPolicy&) {
    auto goal_pose = action_server_->get_current_goal()->goal;

    if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(pnode_->get_logger(), "SingleShot: goal cancelled before plan.");
        action_server_->terminate_all();
        return;
    }

    if (!planOnce(goal_pose)) {
        RCLCPP_ERROR(pnode_->get_logger(), "SingleShot: plan() failed.");
        action_server_->terminate_all();
        return;
    }

    action_server_->succeeded_current();
    RCLCPP_INFO(pnode_->get_logger(), "SingleShot: goal succeeded.");
}

// ------------------------------------------------------------------------------
// executeImpl — TrackingPolicy
// ------------------------------------------------------------------------------

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::executeImpl(TrackingPolicy& policy) {
    auto goal_pose = action_server_->get_current_goal()->goal;
    rclcpp::Rate rate(policy.frequency_hz);

    RCLCPP_INFO(pnode_->get_logger(),
        "Tracking: control loop started at %.1f Hz.", policy.frequency_hz);

    while (rclcpp::ok()) {
        if (action_server_->is_cancel_requested()) {
            RCLCPP_INFO(pnode_->get_logger(), "Tracking: goal cancelled.");
            action_server_->terminate_all();
            return;
        }

        if (!planOnce(goal_pose)) {
            RCLCPP_ERROR(pnode_->get_logger(), "Tracking: plan() failed — aborting.");
            action_server_->terminate_all();
            return;
        }

        // TODO: 接入目标到达判断。
        // 推荐：通过 on_post_plan_ 回调返回 false 来触发退出，
        // 或在此对比 getCurrentPose() 与 goal_pose 的距离阈值。

        rate.sleep();
    }

    action_server_->succeeded_current();
    RCLCPP_INFO(pnode_->get_logger(), "Tracking: goal succeeded.");
}

// ==============================================================================
// bind_map_view 实现
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
template <MapConcept MapT, typename BuilderFn>
requires std::is_invocable_v<BuilderFn, const MapT&> &&
         std::convertible_to<std::invoke_result_t<BuilderFn, const MapT&>,
                             typename ControlLayer<PlannersT, ExecutionPolicy>::MapViewsVariant>
void ControlLayer<PlannersT, ExecutionPolicy>::bind_map_view(
    std::shared_ptr<MapServer<MapT>> server,
    BuilderFn&& builder)
{
    std::function<MapViewsVariant(const MapT&)> erased_builder =
        std::forward<BuilderFn>(builder);
    std::weak_ptr<MapServer<MapT>> weak_server = server;

    map_view_factory_ = [weak_server, erased_builder = std::move(erased_builder)]() -> BoundMapView
    {
        auto srv = weak_server.lock();
        if (!srv) {
            return BoundMapView{};
        }

        auto locked = std::make_shared<LockedMapRO<MapT>>(srv->getLockedReadView());
        MapViewsVariant view_variant = erased_builder(locked->get());
        return BoundMapView{std::move(view_variant), std::move(locked)};
    };
}

} // namespace finenav