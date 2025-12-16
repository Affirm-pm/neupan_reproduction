// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>
#include <concepts>
#include <utility>

#include "math_types.hpp"
#include "finenav_msgs/msg/map_chunk.hpp"
#include "finenav_msgs/msg/plan_result.hpp"

namespace finenav {

/**
 * @brief MapConcept
 * @details
 * Map 被定义为一个具有视窗机制的空间数据容器，M: Postion3D -> DataType
 * * 在外部视角下，核心特性：
 * 1. 读写访问： 提供对特定空间位置的数据访问 (access) 和更新 (update) 接口
 * 2. 视窗机制: 仅允许操作以当前中心的矩形范围内的有效数据，视窗外的数据被视为失效（视窗可以被定义为无穷大）
 * 3. 数据融合: 区别于一般的容器，地图的更新操作通常涉及数据融合，提供自定义融合策略的接口
 *
 */
template <typename T>
concept MapConcept =
    std::default_initializable<T> &&
    std::movable<T> &&
    requires(T map, const Position3D& pos, const typename T::ConfigType& config, const typename T::DataType& data) {
        // --- 基本配置 ---
        typename T::ConfigType;
        typename T::DataType;
        { map.configure(config) } -> std::same_as<bool>; // 配置地图参数

        // --- 读写操作 ---
        { std::as_const(map).access(pos) } -> std::convertible_to<typename T::DataType>;
        { map.integrate(pos, data)} -> std::same_as<bool>; // 数据更新

        // --- 视窗操作 ---
        { std::as_const(map).isInside(pos) } -> std::same_as<bool>; // 检查是否在当前视窗内
        { std::as_const(map).getWindowBounds() } -> std::convertible_to<Region3D>; // 获取当前视窗范围
        { std::as_const(map).getWindowCenter() } -> std::convertible_to<Position3D>; // 获取当前视窗中心
        // map.shift_window(pos, cb); // TODO：移动视窗，等初步测试完之后再进行调整，一步一步测试不用着急
};



// 规划器的统一输出接口：最后由基类输出到自定义消息 fn_interface::msg::plan_result
// 规划器的统一输入接口：PlanningContext (全状态快照)
using TraitMask = uint32_t;

// ========================================================
// 原子特性定义 (Atomic Definitions)
// ========================================================
enum : TraitMask {
    // 空 (用于无输入需求，或不承诺任何额外输出)
    NONE         = 0,
    POSE         = 1 << 0,  // 位置 + 姿态 (Position + Orientation)
    VELOCITY     = 1 << 1,  // 线速度 + 角速度 (Linear + Angular)
    ACCELERATION = 1 << 2,  // 线加速度 + 角加速度
};

// ========================================================
// 常用组合 (Common Combinations)
// ========================================================
// 纯几何路径
static constexpr TraitMask PATH = POSE;
// 完整运动轨迹
static constexpr TraitMask TRAJECTORY = POSE | VELOCITY;
// 全状态轨迹 (包含动力学)
static constexpr TraitMask FULL_STATE = POSE | VELOCITY | ACCELERATION;


/**
 * @brief PlanningContext (规划上下文)
 * @details 包含算法在“这一帧”规划所需的所有不可变数据。
 */
using Twist = int; // 占位符
using Accel = int; // 占位符

struct Trajectory { // TODO: 固定接口为 fn_interface::msg::Trajectory
    // 元数据
    bool success = false;
    uint32_t valid_fields = 0; // 对应 msg 的掩码

    // SoA 数据容器
    std::vector<double> times; // 从规划开始的相对时间 (t0 = 0.0)
    std::vector<Pose> poses;
    std::vector<Twist> twists;
    std::vector<Accel> accels;

    // 辅助函数：检查数据一致性
    size_t size() const { return poses.size(); } // TODO: 有待商榷
    bool empty() const { return poses.empty(); }
};


struct PlanningContext {
    Pose start;
    Pose goal;
    std::optional<Trajectory> reference_path;  // 可选的参考路径
};


// ========================================================
// 主模板 (Primary Template)
// ========================================================
// 默认情况下不定义任何东西。
// 如果用户使用了未注册的 Planner，编译器会报错 "incomplete type PlannerTraits<...>"
template <typename T>
struct PlannerTraits;

// 【关键点】偏特化 (Partial Specialization)
// 语法意思：只要是 AStarPlanner<MapView> 这种形式的类，无论 MapView 是什么，都用这个 Traits
// template <typename MapView>
// struct PlannerTraits<AStarPlanner<MapView>> {
//     // 这里定义的特性与 MapView 具体是什么无关
//     static constexpr auto REQUIRED_INPUTS    = fn_traits::NONE;
//     static constexpr auto GUARANTEED_OUTPUTS = fn_traits::PATH;
// };


/**
 * @brief PlannerConcept TODO：是否考虑改名为 Motion Generator
 * @details
 * Planner 被定义为一个访问地图试图并生成轨迹的算法模块，P: (State, Reference, Map) -> Trajectory
 * * 在外部视角下，核心特性：
 * 1. 特质自省： 暴露它的类型特质，以区分是否需要参考轨迹
 * 2. 不持有地图，只接受地图试图 MapView
 */
template <typename T,  typename MapView>
concept PlannerConcept = std::default_initializable<T> &&
    requires(T planner, MapView& map, const PlanningContext& ctx, const typename T::ConfigType& config) {
        typename T::ConfigType;

        // 规划器必须声明它是哪种类型的，以区分是否需要参考路径
        { PlannerTraits<T>::REQUIRED_INPUTS } -> std::convertible_to<TraitMask>;

        { PlannerTraits<T>::GUARANTEED_OUTPUTS } -> std::convertible_to<TraitMask>;

        // 初始化接口
        { planner.configure(config) } -> std::same_as<void>;

        // 输入地图视图和上下文，返回标准轨迹结构
        { planner.plan(map, ctx) } -> std::same_as<Trajectory>;

};

//  * 规划器的一般形式是(以Astar为例) Astar<MapView> ，其中 MapView 是对 Map 的一种视图包装，由用户功能代码实现；Astar 内部描述了应该如何调用 MapView 的接口
// * 规划器的接口只有一个 plan(), 输入起点位姿和终点位姿，输出 SOA 的规划结果，Local Planner 基类负责将结果打包为 Action 中的相关规范发布

// 再做一个宏定义注册，把 typetraits 绑到 PlannerTraits 管理类的模板特化去


/**
 * @brief LayerConcept
 * @details
*/
template <typename T>
concept LayerTraits = requires {
    typename T::Map;      // 必须包含 MapType
    typename T::Planners;   // 必须包含 PlannerSet
    typename T::Filters;    // 必须包含 FilterSet
}; // TODO: 可能还要补充 PlannerSet 和 FilterSet 的类型检查，之后再说



}
