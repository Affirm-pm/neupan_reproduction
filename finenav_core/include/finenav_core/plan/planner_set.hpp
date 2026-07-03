// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <tuple>
#include <variant>
#include <memory>

namespace finenav::core {

// ==============================================================================
// PlannerSet — 声明一组规划器类型，供 ControlLayer 枚举
// ==============================================================================

template <typename... Ts>
struct PlannerSet {
    using Tuple = std::tuple<Ts...>;
};

// ==============================================================================
// GenerateMapViewsVariant — 从 PlannerSet 推导地图视图 variant 类型
// ==============================================================================

template <typename PlannerT>
struct GetMapViewSharedPtr {
    using type = std::shared_ptr<typename PlannerT::IMapView>;
};

template <typename Tuple> struct ToVariant;

template <typename... Planners>
struct ToVariant<std::tuple<Planners...>> {
    using type = std::variant<typename GetMapViewSharedPtr<Planners>::type...>;
};

// ==============================================================================
// DedupVariant — 从 variant<...> 中移除重复类型
//
// 当多个 Planner 共享同一个 IMapView 类型时（如 RotateShim 与 MPPIController），
// 原始 variant 会包含重复的 shared_ptr<T>，违反 std::variant 的 "每个类型出现
// 且仅出现一次" 约束。DedupVariant 确保生成的 variant 只有唯一类型。
// ==============================================================================

namespace detail {

template <typename T, typename... Ts>
constexpr bool contains_v = (std::is_same_v<T, Ts> || ...);

template <typename AccVariant, typename Remaining>
struct DedupVariantImpl;

// 终止条件：无剩余类型，AccVariant 即为结果
template <typename... Acc>
struct DedupVariantImpl<std::variant<Acc...>, std::tuple<>> {
    using type = std::variant<Acc...>;
};

// 递归：若 First 已存在于 Acc... 中则跳过，否则追加
template <typename... Acc, typename First, typename... Rest>
struct DedupVariantImpl<std::variant<Acc...>, std::tuple<First, Rest...>> {
    using type = typename DedupVariantImpl<
        std::conditional_t<
            contains_v<First, Acc...>,
            std::variant<Acc...>,
            std::variant<Acc..., First>
        >,
        std::tuple<Rest...>
    >::type;
};

template <typename Variant>
struct DedupVariant;

template <typename... Ts>
struct DedupVariant<std::variant<Ts...>> {
    using type = typename DedupVariantImpl<std::variant<>, std::tuple<Ts...>>::type;
};

} // namespace detail

/**
 * @brief 从 PlannerSet 中提取每个规划器的 IMapView，生成 std::variant<shared_ptr<...>...>。
 *
 * 当多个 Planner 共享同一 IMapView 时自动去重，避免 std::variant 中出现重复类型。
 * 用于 ControlLayer::BoundMapView::view_variant 成员的类型推导。
 */
template <typename PlannerSetT>
using GenerateMapViewsVariant = typename detail::DedupVariant<
    typename ToVariant<typename PlannerSetT::Tuple>::type
>::type;

} // namespace finenav::core

