// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/math_types.hpp"
#include "finenav_util/algo_configurator.hpp"
#include "finenav_interface/test_map_params.hpp"

namespace finenav {

class SimpleTestMap {
public:
    using DataType = int;
    using ConfigType = test_map::Params;

    SimpleTestMap() = default;

    void configure(const ConfigType &params) {
        // 假设有 resolution 和 name 参数
        if (params.background.b == 255) { /* 复杂的分支控制逻辑 */}

    }

    DataType access(const Position3D& pos) const {
        (void)pos;
        return 0; // 返回默认值
    }

    void integrate(const Position3D& pos, const DataType& data) {
        (void)pos;
        (void)data;
        // 简单示例，不做实际存储
    }
    bool is_inside(const Position3D& pos) const {
        (void)pos;
        return true; // 始终在范围内
    }
    Region3D get_bounds() const {
        return Region3D{Position3D{0,0,0}, Position3D{10,10,10}}; // 示例边界
    }
    Position3D get_window_center() const {
        return Position3D{5,5,5}; // 示例中心
    }
    // void shift_window(const Position3D& new_center, OffloadCallback<SimpleTestMap> cb) {
    //     (void)new_center;
    //     (void)cb;
    //     // 简单示例，不做实际操作
    // }

    void debug_print() const {
        RCLCPP_INFO(rclcpp::get_logger("SimpleTestMap"), "SimpleTestMap debug_print called");
    }

};
}

FINENAV_REGISTER_ALGO_CONFIG(finenav::SimpleTestMap, test_map)