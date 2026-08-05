// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_core/detail/map_server_infra.hpp"

#include <chrono>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::core::detail {

void declareMapServerParameters(
    rclcpp::Node& pnode,
    double& update_rate_hz,
    double& shift_rate_hz,
    double& shift_margin_ratio)
{
    // update_rate_hz
    {
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.description = "Sensor fusion update rate (Hz). Must be > 0.0.";
        pnode.declare_parameter<double>("update_rate_hz", update_rate_hz, desc);
        update_rate_hz = pnode.get_parameter("update_rate_hz").as_double();
        if (update_rate_hz <= 0.0) {
            FINENAV_LOG_WARN(pnode.get_logger(),
                "update_rate_hz must be > 0.0 (got {:.1f}). Falling back to 10.0.",
                update_rate_hz);
            update_rate_hz = 10.0;
        }
    }
    // shift_rate_hz
    {
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.description = "Map window shift check rate (Hz). Must be > 0.0.";
        pnode.declare_parameter<double>("shift_rate_hz", shift_rate_hz, desc);
        shift_rate_hz = pnode.get_parameter("shift_rate_hz").as_double();
        if (shift_rate_hz <= 0.0) {
            FINENAV_LOG_WARN(pnode.get_logger(),
                "shift_rate_hz must be > 0.0 (got {:.1f}). Falling back to 50.0.",
                shift_rate_hz);
            shift_rate_hz = 50.0;
        }
    }
    // shift_margin_ratio
    {
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.description =
            "Distance-from-centre ratio triggering a window shift "
            "(0 = centre only, 1 = at edge). Must be in [0.0, 1.0].";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = 0.0;
        range.to_value   = 1.0;
        range.step       = 0.0;
        desc.floating_point_range = {range};
        pnode.declare_parameter<double>("shift_margin_ratio", shift_margin_ratio, desc);
        shift_margin_ratio = pnode.get_parameter("shift_margin_ratio").as_double();
        if (shift_margin_ratio < 0.0) {
            FINENAV_LOG_WARN(pnode.get_logger(),
                "shift_margin_ratio must be >= 0.0 (got {:.2f}). Falling back to 0.0.",
                shift_margin_ratio);
            shift_margin_ratio = 0.0;
        }
        if (shift_margin_ratio > 1.0) {
            FINENAV_LOG_WARN(pnode.get_logger(),
                "shift_margin_ratio must be <= 1.0 (got {:.2f}). Falling back to 1.0.",
                shift_margin_ratio);
            shift_margin_ratio = 1.0;
        }
    }
}

MapServerInfra buildMapServerInfra(
    rclcpp::Node::SharedPtr parent_node,
    const std::string& server_name,
    double update_rate_hz,
    double shift_rate_hz,
    double shift_margin_ratio)
{
    MapServerInfra infra;
    infra.update_rate_hz     = update_rate_hz;
    infra.shift_rate_hz      = shift_rate_hz;
    infra.shift_margin_ratio = shift_margin_ratio;

    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        parent_node->get_parameter("use_sim_time").as_bool());
    infra.pnode = rclcpp::Node::make_shared(server_name, parent_node->get_name(), pnode_opts);
    infra.pnode_thread = std::make_unique<finenav::util::NodeThread>(infra.pnode);

    declareMapServerParameters(
        *infra.pnode, infra.update_rate_hz, infra.shift_rate_hz, infra.shift_margin_ratio);

    auto update_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / infra.update_rate_hz));
    infra.update_timer = rclcpp::create_timer(
        infra.pnode, infra.pnode->get_clock(), update_period,
        [pnode = infra.pnode]() {
            FINENAV_LOG_WARN_ONCE(pnode->get_logger(),
                "MapServer update timer fired but runUpdateCycle not yet bound.");
        });

    return infra;
}

}  // namespace finenav::core::detail
