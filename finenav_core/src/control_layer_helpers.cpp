// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_core/detail/control_layer_infra.hpp"

#include "finenav_core/plan/execution_policy.hpp"

namespace finenav::core::detail {

ControlLayerInfra buildControlLayerInfra(
    rclcpp::Node::SharedPtr parent_node,
    const std::string& layer_name,
    const ExecutionPolicyVariant& policy)
{
    ControlLayerInfra infra;

    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        parent_node->get_parameter("use_sim_time").as_bool());
    infra.pnode = rclcpp::Node::make_shared(layer_name, parent_node->get_name(), pnode_opts);
    infra.pnode_thread = std::make_unique<finenav::util::NodeThread>(infra.pnode);

    std::visit([&pnode = infra.pnode](auto& p) {
        if constexpr (requires { p.declare_parameters(*pnode); }) {
            p.declare_parameters(*pnode);
        }
    }, policy);

    infra.worker = std::make_unique<finenav::util::TaskWorker>(infra.pnode, layer_name);

    return infra;
}

}  // namespace finenav::core::detail
