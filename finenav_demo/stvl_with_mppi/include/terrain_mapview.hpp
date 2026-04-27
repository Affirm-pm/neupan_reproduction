// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_collision_model/obb_collision_model.hpp"
#include "grid_map.hpp"
#include "terrain_analysis.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include <cmath>
using namespace mppi;
using std::chrono_literals::operator"" ms;

extern finenav::OBBCollisionModel collision_model;

class SimpleGridMapView : public nav2_mppi_controller::IMapView {
public:
    explicit SimpleGridMapView(const finenav::GridMap<Voxel>& map, rclcpp::Node::SharedPtr node, std::shared_ptr<TerrainAnalyzer> terrain_analyzer);

    bool isTrackingUnknown() const override;

    bool considerFootprint() const override;

    bool isCollision(float x, float y, float theta) const override;

    int getCost(const finenav::Position3D& pos) const override;

    float getRadius() const override;

    float costAtPose(float x, float y, float theta) const override;

    std::string getBaseFrameID() const override;

    void publishLocalMap();

private:
    std::function<bool(const finenav::Position3D&)> CollisionRule() const;

    int ComputeCost(int distance) const;

    finenav::Pose createPose2D(double x, double y, double theta) const;

    std::shared_ptr<TerrainAnalyzer> terrain_analyzer_;
    const finenav::GridMap<Voxel>& map_;
    double inscribed_radius_;
    bool is_tracking_unknown_{false};
    bool consider_footprint_{true};
    Eigen::ArrayXXf ground_array_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pub_;

    finenav::OBBCollisionModel collision_model_;
};
