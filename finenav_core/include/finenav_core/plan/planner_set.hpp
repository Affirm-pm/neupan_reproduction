// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <tuple>
#include <variant>
#include <memory>

namespace finenav::core {

/**
 * @brief PlannerSet — declares a set of planner types for a ControlLayer.
 * Usage:
 * @code
 *   using MyPlanners = PlannerSet<PlannerA, PlannerB}>;
 *   ControlLayer<MyPlanners> layer(node, "my_layer");
 * @endcode
 */
template <typename... Ts>
struct PlannerSet {
    using Tuple = std::tuple<Ts...>;
};

/**
 * @brief GenerateMapViewsVariant — builds a deduplicated std::variant of all the
 * IMapView shared_ptr types declared by the planners in PlannerSetT.
 *
 * @example   PlannerSet<A, B, C> where B::IMapView == C::IMapView
 *
 *   Step 1 — ToVariant:
 *     variant< shared_ptr<A::IMapView>,
 *              shared_ptr<B::IMapView>,
 *              shared_ptr<C::IMapView> >
 *
 *   Step 2 — DedupVariant:
 *     variant< shared_ptr<A::IMapView>,
 *              shared_ptr<B::IMapView> >
 */

// tuple<Ts...> → variant<Ts...>
template <typename Tuple> struct ToVariant;

template <typename... Planners>
struct ToVariant<std::tuple<Planners...>> {
    using type = std::variant<std::shared_ptr<typename Planners::IMapView>...>;
};

// Recursively removes duplicate types from a variant.
namespace detail {

template <typename T, typename... Ts>
constexpr bool contains_v = (std::is_same_v<T, Ts> || ...);

// Base case: no more types to process.
template <typename AccVariant, typename RemainingTuple>
struct DedupVariantImpl;

template <typename... Acc>
struct DedupVariantImpl<std::variant<Acc...>, std::tuple<>> {
    using type = std::variant<Acc...>;
};

// Recursive step: check First against Acc, append if new, then recurse on Rest.
template <typename... Acc, typename First, typename... Rest>
struct DedupVariantImpl<std::variant<Acc...>, std::tuple<First, Rest...>> {
    using type = typename DedupVariantImpl<
        std::conditional_t<
            contains_v<First, Acc...>,
            std::variant<Acc...>,              // duplicate → drop
            std::variant<Acc..., First>        // new     → append
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
 * @brief Public entry point: PlannerSetT → deduplicated variant of IMapView shared_ptrs.
 * @tparam PlannerSetT  A PlannerSet<...> instantiation.
 */
template <typename PlannerSetT>
using GenerateMapViewsVariant = typename detail::DedupVariant<
    typename ToVariant<typename PlannerSetT::Tuple>::type
>::type;

} // namespace finenav::core
