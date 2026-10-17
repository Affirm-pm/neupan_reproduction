// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// FineNavLocalizer entry point for eval — never linked into production.

#include <rclcpp/rclcpp.hpp>
#include "finenav_localizer/finenav_localizer.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto opts = rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true);
    opts.append_parameter_override("use_sim_time", true);

    auto node = rclcpp::Node::make_shared("localizer_eval_node", opts);
    finenav::localizer::FineNavLocalizer localizer(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
