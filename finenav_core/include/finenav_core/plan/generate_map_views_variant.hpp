// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <tuple>
#include <variant>

namespace finenav::core {

/**
 * @brief GenerateMapViewsVariant — builds a deduplicated std::variant of
 * IMapView shared_ptr types declared by the given set of planners.
 *
 * @example   GenerateMapViewsVariant<A, B, C> where B::IMapView == C::IMapView
 *
 *   Step 1 — raw variant:
 *     std::variant< shared_ptr<A::IMapView>,
 *                   shared_ptr<B::IMapView>,
 *                   shared_ptr<C::IMapView> >
 *
 *   Step 2 — DedupVariant:
 *     std::variant< shared_ptr<A::IMapView>,
 *                   shared_ptr<B::IMapView> >
 *
 * @tparam Planners  The planner types whose IMapViews are collected.
 */

// Recursively removes duplicate types from a variant.
namespace detail {

template <typename T, typename... Ts>
constexpr bool contains_v = (std::is_same_v<T, Ts> || ...);

template <typename AccVariant, typename RemainingTuple>
struct DedupVariantImpl;

template <typename... Acc>
struct DedupVariantImpl<std::variant<Acc...>, std::tuple<>> {
    using type = std::variant<Acc...>;
};

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
 * @brief  Variadic entry point:  Planners...  →  deduplicated IMapView variant.
 * @tparam Planners  One or more planner types.
 */
template <typename... Planners>
using GenerateMapViewsVariant = typename detail::DedupVariant<
    std::variant<std::shared_ptr<typename Planners::IMapView>...>
>::type;

}  // namespace finenav::core
