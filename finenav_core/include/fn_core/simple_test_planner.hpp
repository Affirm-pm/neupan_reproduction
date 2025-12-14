// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

/// 暂时为了能够进行编译测试的简单实现，后续会迁移到独立的 package
#pragma once

#include "local_planner_base.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace finenav {

// 简单的测试地图类
class SimpleTestMap {
public:
    using DataType = int;

    SimpleTestMap() = default;


    void configure() {
        // 简单示例，不做实际配置
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
    void shift_window(const Position3D& new_center, OffloadCallback<SimpleTestMap> cb) {
        (void)new_center;
        (void)cb;
        // 简单示例，不做实际操作
    }

    void debug_print() const {
        RCLCPP_INFO(rclcpp::get_logger("SimpleTestMap"), "SimpleTestMap debug_print called");
    }

};

// 简单的测试规划器类
class SimpleTestPlanner {
public:
    // 定义静态常量 name
    // constexpr 表示编译期常量，更高效
    static constexpr auto name = "SimpleTest";


    SimpleTestPlanner() = default;

    void configure(rclcpp::Node* node, const std::string& namespace_) {

        // 简单示例，不做实际配置
    }

    std::vector<Pose> plan(const Pose& start, const Pose& goal, const SimpleTestMap& map) {
        (void)start;
        (void)goal;
        (void)map;
        // 返回空路径
        return std::vector<Pose>();
    }
};

struct LocalLayerConfig {
    using Map = SimpleTestMap;
    using Planners = PlannerSet<SimpleTestPlanner>;
    using Filters = FilterSet<>;
};


// 测试子类
class SimpleLocalLayer : public LocalPlannerBase<LocalLayerConfig> {
public:

    explicit SimpleLocalLayer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : LocalPlannerBase<LocalLayerConfig>(options) {
        this->map_instance_.debug_print();

    }

};

}
