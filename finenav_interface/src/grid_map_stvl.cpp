// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-4-2.
//
#include "grid_map.hpp"
#include "simpleTestPlanner.hpp"
#include "finenav_manager.hpp"
#include "map_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <vector>

using namespace finenav;

struct Pointcloud {
    float x;
    float y;
    float z;
};

// 体素中存储：z 轴高度、是否静态、最近一次更新时间戳（ROS2 时间）
struct Voxel {
    float        z_height  = NAN;
    bool         is_static = false;
    rclcpp::Time timestamp{};   // 使用 node->get_clock()->now() 赋值
};
// ============================ 动态栅格管理相关 ============================

// 使用 Position 作为 key 的哈希器和相等判断，只在本 demo 中使用
struct PositionHash
{
    std::size_t operator()(const Position & p) const noexcept
    {
        // Position 是 Eigen::Vector3d，直接对三维坐标做 hash
        std::size_t hx = std::hash<double>{}(p.x());
        std::size_t hy = std::hash<double>{}(p.y());
        std::size_t hz = std::hash<double>{}(p.z());

        auto hash_combine = [](std::size_t seed, std::size_t v) {
            seed ^= v + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            return seed;
        };

        std::size_t seed = 0;
        seed = hash_combine(seed, hx);
        seed = hash_combine(seed, hy);
        seed = hash_combine(seed, hz);
        return seed;
    }
};

struct PositionEqual
{
    bool operator()(const Position & a, const Position & b) const noexcept
    {
        // 由于 Position 将通过 GridMap::getPosition(Index) 得到，理论上是栅格中心的精确坐标，
        // 这里直接用完全相等判断。如果将来引入插值，可在此处添加容差。
        return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
    }
};

using DynamicPosSet = std::unordered_set<Position, PositionHash, PositionEqual>;

// 全局（本文件作用域）动态栅格集合与超时时间，仅用于 demo
static DynamicPosSet g_dynamic_cells;          // 存储动态栅格的中心 Position
static double        g_decay_time_sec = 10.0;   // 超时阈值，单位：秒

static const rclcpp::Logger LOGGER = rclcpp::get_logger("finenav_demo");

using GlobalPlanners = PlannerSet<SimpleTestPlanner>;
using LocalLayers = PlannerSet<SimpleTestPlanner>;
using ReactivePlanners = PlannerSet<SimpleTestPlanner>;


bool numisfinite(float x) {
    if(x==std::numeric_limits<float>::infinity() || x==-std::numeric_limits<float>::infinity() || std::isnan(x)) {
        return false;
    }
    return true;
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("finenav_demo");

    auto finenav_ptr = std::make_shared<FineNavManager>(node);

    auto global_layer = finenav_ptr->createControlLayer<GlobalPlanners>("global_layer");
    auto local_layer = finenav_ptr->createControlLayer<LocalLayers>("local_layer");
    auto reactive_layer = finenav_ptr->createControlLayer<ReactivePlanners>("reactive_layer");

    finenav_ptr->registerPipeline(FineNavManager::PipelineIO{}, global_layer, local_layer, reactive_layer);
    auto map_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", 10);
    // ==============================================================================
    // 测试 MapServer 的完整功能 (Hook机制 + 数据源Push模式)
    // ==============================================================================

    // 1. 初始化一个地图服务器，指定更新频率 1.0Hz
    MapServer<GridMap<Voxel>> map_server(node, "test_map_server", 10.0);

    // 2. 注册预处理 Hook (在应用融合数据前执行)
    map_server.registerPreUpdateHook([node](GridMap<Voxel>& map) {
        rclcpp::Time now = node->get_clock()->now();

        // 遍历所有当前记录为“动态”的栅格中心 Position，根据时间戳判断是否需要清除
        for (auto it = g_dynamic_cells.begin(); it != g_dynamic_cells.end(); ) {
            const Position & pos = *it;

            // 如果位置已不在地图有效范围内，直接移除
            if (!map.isInside(pos)) {
                it = g_dynamic_cells.erase(it);
                continue;
            }

            Voxel & cell = map.atPosition(pos);

            // 使用 ROS2 时间计算持续时间
            rclcpp::Duration age = now - cell.timestamp;
            if (age.seconds() > g_decay_time_sec) {
                // 超时：清除该体素的动态信息，这里简单重置为默认值
                cell = Voxel{NAN, false, rclcpp::Time{}};
                it   = g_dynamic_cells.erase(it);
            } else {
                ++it;
            }
        }

        RCLCPP_INFO(LOGGER, ">> [Hook] PreUpdateHook: dynamic cells = %zu", g_dynamic_cells.size());
    });

    // 3. 注册后处理 Hook (在应用融合数据后执行)
    map_server.registerPostUpdateHook(
        [node, map_cloud_pub](const GridMap<Voxel>& map)
        {
            auto now = node->now();
            std::vector<Index> valid_indices;
            map.selectCellsByCondition(valid_indices, [](const Voxel& v) {
                return std::isfinite(v.z_height);
            });
            // std::cout<<"valid_indices size: "<<valid_indices.size()<<std::endl;
            if (valid_indices.empty()) return;

            sensor_msgs::msg::PointCloud2 cloud;
            cloud.header.stamp = now;
            cloud.header.frame_id = "odom";
            cloud.height = 1;
            cloud.width = valid_indices.size();

            sensor_msgs::PointCloud2Modifier modifier(cloud);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(valid_indices.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

            for (const auto& idx : valid_indices) {
                const auto pos = map.getPosition(idx);
                const auto& cell = map.at(idx);

                *iter_x = pos.x();
                *iter_y = pos.y();
                *iter_z = cell.z_height;

                ++iter_x;
                ++iter_y;
                ++iter_z;
            }

            map_cloud_pub->publish(cloud);
            RCLCPP_INFO(LOGGER, "<< [Hook] PostUpdateHook triggered.");

        }
    );

    // 4. 初始化策略并注册数据注射器
    MapServer<GridMap<Voxel>>::SourceBufferPolicy policy;
    policy.max_buffer_size = 50000;
    policy.observation_keep_time_sec = 2.0;
    auto laser_injector = map_server.addUpdateSource<Pointcloud>(
        "front_laser",
        policy,
        [node](const Pointcloud& msg, GridMap<Voxel>& map) {
            rclcpp::Time now = node->get_clock()->now();
            // 演示：假设激光击中地图中心 (0,0) 处的某个体素，只更新一次
            // 实际工程中应该根据传感器模型计算出命中位置
            Position hit_pos = Position(msg.x , msg.y, msg.z); // TODO: 根据 msg 计算实际命中位置

            if (!map.isInside(hit_pos)) {
                return;
            }

            Index  idx  = map.getIndex(hit_pos);
            Voxel & cell = map.at(idx);

            if (!cell.is_static) {
                cell.z_height  = msg.z;   // 这里用 range 代替 z，仅作示意
                cell.timestamp = now;         // 记录 ROS2 时间
                // 使用 Position 作为 key 记录为动态栅格（哈希表自动去重）
                Position center_pos = map.getPosition(idx);
                g_dynamic_cells.insert(center_pos);
                std::cout<<"Injected dynamic cell at ("<<center_pos.x()<<", "<<center_pos.y()<<", "<<center_pos.z()<<") with z_height = "<<cell.z_height<<std::endl;
            }
        }
    );

    // 新增：地图点云可视化发布器（手动触发发布）


    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_for_lio",
        rclcpp::SensorDataQoS(),
        [node, laser_injector, &map_server, map_cloud_pub](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
            rclcpp::Time now = node->now();

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                if(numisfinite(*iter_x) && numisfinite(*iter_y) && numisfinite(*iter_z)) {
                    Pointcloud p;
                    p.x = *iter_x;
                    p.y = *iter_y;
                    p.z = *iter_z;
                    laser_injector(std::move(p), now);
                }
            }

            // // 在 main 的回调里手动调用（非周期）
            // auto locked_map = map_server.getLockedReadView();
            // publishMapAsPointCloud(locked_map, map_cloud_pub, "map", now);
        }
    );

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

