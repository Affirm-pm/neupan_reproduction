// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/control_layer.hpp"

#include "finenav_msgs/msg/map_chunk.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace finenav {

template <LayerTraits Traits>
class LocalLayer : public ControlLayer<Traits> {
public:

    static ControlLayerConfig getDefaultConfig() {
        return ControlLayerConfig{
            .task_action = "follow_path",
            .input_topic = "",  // 表示不需要输入
            .output_topic = "local_plan",
            .mode = ControlMode::OneShot
        };
    }

    explicit LocalLayer(const rclcpp::NodeOptions & options)
        : ControlLayer<Traits>("local_layer", options, getDefaultConfig()) {

        RCLCPP_INFO(this->get_logger(), "LocalLayer initialized");

        // 传感器数据源
        pointcloud_sub_ = this->template create_subscription<sensor_msgs::msg::PointCloud2>(
            "pointcloud", 10,  [this](const sensor_msgs::msg::PointCloud2 & msg) {
                RCLCPP_INFO(this->get_logger(), "Subscribing to points");
                /****** 在这里处理地图更新，“喂养”基类持有的地图实例 ******/
                // ... this->map_instance.updata)_;
            });

        // 地图交互端口
        // TODO: offload_pub_ 和 load_sub_ 都需要一个对应的 service 向服务器请求，只有服务器回复 yes，才用 topic 进行数据传输

        load_sub_ = this->template create_subscription<finenav_msgs::msg::MapChunk>(
            "map_load", 10,
            [this](const finenav_msgs::msg::MapChunk::SharedPtr msg) {
                // server 发过来了 MapChunk，其中包含了 (x, y, z, filed1, field2, ...)
                RCLCPP_INFO(this->get_logger(), "Received map chunk to load");
                /****** 在这里处理地图更新 ******/
                // TODO: 多个地图更新源，这里需要时序检测，责任应该给控制层还是 map 库？
            });

        // TODO: 该怎么知道应该如何用手上的map装填MapChunk呢？
        offload_pub_ = this->template create_publisher<finenav_msgs::msg::MapChunk>("map_offload", 10);

        // TODO: 需要单独开一个线程，持续监测 TF 定位，一旦定位变化，调用视窗移动函数。这是 LocalLayer 控制层独有的配置
    }



private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    // 地图交互
    rclcpp::Publisher<finenav_msgs::msg::MapChunk>::SharedPtr offload_pub_;
    rclcpp::Subscription<finenav_msgs::msg::MapChunk>::SharedPtr load_sub_;

};

};

