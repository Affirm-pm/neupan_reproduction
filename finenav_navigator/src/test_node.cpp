// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include "navigator.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Navigator 遵循 NodeThread 模式，构造时传入宿主节点，内部自动建立 pnode_ + NodeThread。
    auto host_node = rclcpp::Node::make_shared("finenav_test_host");
    finenav::Navigator navigator(host_node);

    // Navigator 已在独立线程中 spin；主线程只需保持 host_node 存活。
    rclcpp::spin(host_node);

    rclcpp::shutdown();
    return 0;
}
