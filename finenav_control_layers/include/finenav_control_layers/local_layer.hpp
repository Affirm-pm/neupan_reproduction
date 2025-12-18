// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/control_layer.hpp"

#include "finenav_msgs/msg/map_chunk.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // TODO: 需要好好处理尖括号和双引号

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace finenav {

template <LayerTraits Traits>
class LocalLayer : public ControlLayer<Traits> {
public:

    // TODO: 处理局部目标，距离路径裁剪，就类似于 controller_server 在调用 controller 前要做的事情，这是作为 LocalLayer 不可分割的语义

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

        // TODO: 需要单独开一个线程，持续监测 TF 定位，一旦定位变化，调用 OnMapMoved()
    }

protected:
    void insertSensorScan(const sensor_msgs::msg::PointCloud2 & msg) {
        /****** 在这里处理地图更新，“喂养”基类持有的地图实例 ******/
        RCLCPP_INFO(this->get_logger(), "Subscribing to points");
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromROSMsg(msg, pc);
        for (const auto& p : pc) {
            Position pos{p.x, p.y,p.z};
            if (!this->map_instance_.isInside(pos) ) {  // TODO: 多个读写线程，注意上锁
                static int cnt = 0;
                cnt += 1;
                std::cout << "[LocalLayer] Point " << cnt << " is outside the map window: "
                          << " (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
            }
        }
    }

    void updateFromOtherMap(const finenav_msgs::msg::MapChunk & msg) {
        // server 发过来了 MapChunk，其中包含了 (x, y, z, filed1, field2, ...)
        RCLCPP_INFO(this->get_logger(), "Received map chunk to load");
        /****** 在这里处理地图更新 ******/
        // TODO: 多个地图更新源，这里需要时序检测，责任应该给控制层还是 map 库？
    }

    virtual void onMapMoved(const Position& new_center) {
        RCLCPP_INFO(this->get_logger(), "Map moved to new center: (%.2f, %.2f, %.2f)",
                    new_center.x(), new_center.y(), new_center.z());
        // this->map_instance_.shift_window(new_center, nullptr);
    }

};

}

