// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <string_view>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/common/algo_registry.hpp"
#include "finenav_core/common/param_pollable_keeper.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

namespace finenav::core {
namespace detail {

/**
 * @brief Compile-time registration slot for an algorithm (map or planner).
 * @tparam AlgoT  Algorithm type.
 */
template <typename AlgoT>
struct RegistrationSlot {};

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

/**
 * @brief Shared load-body used by all registration macros.
 *
 * @tparam AlgoT     Algorithm type.
 * @tparam ParamsT   generate_parameter_library param struct (PARAM_NS::Params).
 * @tparam ListenerT generate_parameter_library ParamListener (PARAM_NS::ParamListener).
 */
template <typename AlgoT, typename ParamsT, typename ListenerT>
std::shared_ptr<IParamPollable> makeParamPollable(
    rclcpp::Node::SharedPtr node, AlgoT& algo, std::string_view algo_name) {
    // Configure parameters
    auto listener = std::make_shared<ListenerT>(node, std::string{algo_name});
    auto params = listener->get_params();
    algo.configure(params);

    // Inject resource to algo class
    injectLogger(node, algo, algo_name);
    auto viz = injectVisualizer(node, algo);

    struct ParamPollable : public IParamPollable {
        std::shared_ptr<ListenerT> listener;
        std::shared_ptr<finenav::viz::FineNavVisualizer> viz;
        AlgoT* algo_ptr;
        ParamsT params;
        std::string algo_name;

        ParamPollable(std::shared_ptr<ListenerT> l,
                      std::shared_ptr<finenav::viz::FineNavVisualizer> v,
                      AlgoT* a, ParamsT p, std::string n)
            : listener(std::move(l)), viz(std::move(v)),
              algo_ptr(a), params(std::move(p)), algo_name(std::move(n)) {}

        void poll() override {
            if (listener->is_old(params)) {
                params = listener->get_params();
                RCLCPP_DEBUG(rclcpp::get_logger(algo_name),
                    "Algo '{}': dynamic params updated, forwarding to configure().",
                    algo_name);
                algo_ptr->configure(params);
            }
        }
    };

    return std::make_shared<ParamPollable>(listener, viz, &algo, params, std::string{algo_name});
}

/**
 * @brief Adapts the parameter wiring to the registry's type-erased ConfigLoader.
 *
 * @tparam AlgoT     Algorithm type.
 * @tparam ParamsT   generate_parameter_library param struct (PARAM_NS::Params).
 * @tparam ListenerT generate_parameter_library ParamListener (PARAM_NS::ParamListener).
 */
template <typename AlgoT, typename ParamsT, typename ListenerT>
AlgoRegistry::ConfigLoader makeConfigLoader(std::string_view algo_name) {
    return [algo_name](rclcpp::Node::SharedPtr node, void* instance)
        -> std::shared_ptr<IParamPollable> {
        return makeParamPollable<AlgoT, ParamsT, ListenerT>(
            node, *static_cast<AlgoT*>(instance), algo_name);
    };
}

}  // namespace detail
}  // namespace finenav::core
