// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

namespace finenav::core {

/**
 * @brief Hooks an algorithm (map or planner) into the FineNav-Engine parameter system.
 *
 * Every algorithm type needs a corresponding AlgoConfigurator specialisation. The specialisation tells the framework:
 *   - its parameter namespace
 *   - how to create a ParamListener and call algo.configure()
 *   - how to register a dynamic-reconfigure callback
 *
 * Use FINENAV_REGISTER_ALGO_CONFIG or FINENAV_REGISTER_TEMPLATE_ALGO_CONFIG
 * rather than writing a specialisation by hand.
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

    /// Default (no-op) load.  Algorithms that are never configured will hit this fallback.
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr, AlgoT&) {
        return nullptr;
    }
};

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
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr node, ALGO_TYPE& algo) { \
        auto listener = std::make_shared<PARAM_NS::ParamListener>(node, #PARAM_NS); \
        algo.configure(listener->get_params()); \
        listener->setUserCallback([&algo](const auto& params) { \
            algo.configure(params); \
        }); \
        return listener; \
    } \
}; \
}

/**
 * @brief Hooks a template algorithm (map<T, ...> or planner<T, ...>) into the FineNav-Engine parameter system.
 * @param TEMPLATE_ALGO Algorithm template.
 * @param PARAM_NS It is a human-readable identifier for this algorithm. It should be the same with the parameter namespace for this algorithm.
 */
#define FINENAV_REGISTER_TEMPLATE_ALGO_CONFIG(TEMPLATE_ALGO, PARAM_NS) \
namespace finenav::core { \
template <typename... Args> \
struct AlgoConfigurator<TEMPLATE_ALGO<Args...>> { \
    static constexpr std::string_view name = #PARAM_NS; \
    static std::shared_ptr<void> load(rclcpp::Node::SharedPtr node, TEMPLATE_ALGO<Args...>& algo) { \
        auto listener = std::make_shared<PARAM_NS::ParamListener>(node, #PARAM_NS); \
        algo.configure(listener->get_params()); \
        listener->setUserCallback([&algo](const auto& params) { \
            algo.configure(params); \
        }); \
        return listener; \
    } \
};\
}\

