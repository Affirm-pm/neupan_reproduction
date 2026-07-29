// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/param_handle_keeper.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

namespace finenav::core {

/**
 * @brief Hooks an algorithm (map or planner) into the FineNav-Engine parameter system.
 *
 * Every algorithm type needs a corresponding AlgoConfigurator specialisation.
 * The specialisation tells the framework:
 *   - its parameter namespace
 *   - how to create a ParamListener and call algo.configure()
 *   - how to create a pollable handle for thread-safe dynamic parameter updates
 *
 * Use FINENAV_REGISTER_ALGO_CONFIG rather than writing a specialisation by hand.
 *
 * @tparam AlgoT  Algorithm type.
 */

// TODO： Algo Configurator
// 1. AlgoT编译保护，只允许 map 和 planner
// 2. 怎么搞参数热加载
// 3. 如果重名怎么办
template <typename AlgoT>
struct AlgoConfigurator {
    /// Human-readable identifier for this algorithm.
    static constexpr std::string_view name = "unknown_algo";

    /// Default (no-op) load.  Algorithms that are never configured hit this fallback.
    static std::shared_ptr<IParamPollable> load(rclcpp::Node::SharedPtr, AlgoT&) {
        return nullptr;
    }
};

// ── Injection helpers (function templates so if constexpr works) ───────

template <typename AlgoT>
void injectLogger(rclcpp::Node::SharedPtr node, AlgoT& algo, std::string_view name) {
    if constexpr (requires { algo.setLogger(rclcpp::Logger{}); }) {
        algo.setLogger(node->get_logger().get_child(std::string{name}));
    }
}

template <typename AlgoT>
std::shared_ptr<finenav::viz::FineNavVisualizer> injectVisualizer(
    rclcpp::Node::SharedPtr node, AlgoT& algo) {
    if constexpr (requires { algo.setVisualizer(
            std::declval<finenav::viz::FineNavVisualizer&>()); }) {
        auto viz = std::make_shared<finenav::viz::FineNavVisualizer>(node);
        algo.setVisualizer(*viz);
        return viz;
    }
    return nullptr;
}

}  // namespace finenav::core

/**
 * @brief Hooks an algorithm (map or planner) into the FineNav-Engine parameter system.
 * @param ALGO_TYPE Algorithm type.
 * @param PARAM_NS It is a human-readable identifier for this algorithm. It should be the same with the parameter namespace for this algorithm.
 */
#define FINENAV_REGISTER_ALGO_CONFIG(ALGO_TYPE, PARAM_NS) \
namespace finenav::core { \
template <> \
struct AlgoConfigurator<ALGO_TYPE> { \
    static constexpr std::string_view name = #PARAM_NS; \
    static std::shared_ptr<IParamPollable> load(rclcpp::Node::SharedPtr node, ALGO_TYPE& algo) { \
        auto listener = std::make_shared<PARAM_NS::ParamListener>(node, #PARAM_NS); \
        auto params = listener->get_params(); \
        algo.configure(params); \
        injectLogger(node, algo, name); \
        auto viz = injectVisualizer(node, algo); \
        struct Handle : public IParamPollable { \
            std::shared_ptr<PARAM_NS::ParamListener> listener; \
            std::shared_ptr<finenav::viz::FineNavVisualizer> viz; \
            ALGO_TYPE* algo_ptr; \
            PARAM_NS::Params params; \
            std::string algo_name; \
            Handle(std::shared_ptr<PARAM_NS::ParamListener> l, \
                   std::shared_ptr<finenav::viz::FineNavVisualizer> v, \
                   ALGO_TYPE* a, PARAM_NS::Params p, std::string n) \
                : listener(std::move(l)), viz(std::move(v)), \
                  algo_ptr(a), params(std::move(p)), algo_name(std::move(n)) {} \
            void poll() override { \
                if (listener->is_old(params)) { \
                    params = listener->get_params(); \
                    RCLCPP_DEBUG(rclcpp::get_logger(algo_name), \
                        "Algo '{}': dynamic params updated, forwarding to configure().", \
                        algo_name); \
                    algo_ptr->configure(params); \
                } \
            } \
        }; \
        return std::make_shared<Handle>(listener, viz, &algo, params, std::string{name}); \
    } \
}; \
}
