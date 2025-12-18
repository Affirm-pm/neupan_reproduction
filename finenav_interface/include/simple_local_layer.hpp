// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_control_layers/local_layer.hpp"
#include "grid_map.hpp"

#include "finenav_interface/test_map_params.hpp"
#include "finenav_util/algo_configurator.hpp"

namespace finenav {


// 简单的测试地图类
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

// 简单的测试规划器类
class SimpleTestPlanner {
public:
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
    // using Map = SimpleTestMap;
    using Map = finenav::GridMap<float>;
    using Planners = PlannerSet<SimpleTestPlanner>;
    using Filters = FilterSet<>;
};


bool custom_map_integrate(const Position& position, const LocalLayerConfig::Map::DataType& data) {
    // e.g. 贝叶斯更新
    // 是否提供一个允许自定义融合策略的接口，不由 FineNav 核心规定，而由每个具体的 map 库自治
    return true;
}

// 测试子类
class SimpleLocalLayer : public LocalLayer<LocalLayerConfig> {
public:

    explicit SimpleLocalLayer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : LocalLayer<LocalLayerConfig>(options) {
        // this->map_instance_.debug_print();

        // 传感器数据源
        pointcloud_sub_ = this->template create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10,  [this](const sensor_msgs::msg::PointCloud2 & msg) {
                this->insertSensorScan(msg);
            });

        // 地图交互端口
        // TODO: offload_pub_ 和 load_sub_ 都需要一个对应的 service 向服务器请求，只有服务器回复 yes，才用 topic 进行数据传输

        load_sub_ = this->template create_subscription<finenav_msgs::msg::MapChunk>(
            "map_load", 10,
            [this](const finenav_msgs::msg::MapChunk::SharedPtr msg) {
                this->updateFromOtherMap(*msg);
            });

        // TODO: 该怎么知道应该如何用手上的map装填MapChunk呢？
        offload_pub_ = this->template create_publisher<finenav_msgs::msg::MapChunk>("map_offload", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    // 地图交互
    rclcpp::Publisher<finenav_msgs::msg::MapChunk>::SharedPtr offload_pub_;
    rclcpp::Subscription<finenav_msgs::msg::MapChunk>::SharedPtr load_sub_;

};

}

FINENAV_REGISTER_ALGO_CONFIG(finenav::SimpleTestMap, test_map)