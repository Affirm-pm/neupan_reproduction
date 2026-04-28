// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

//
// Created by fins on 26-4-27.
//

#include "terrain_mapview.hpp"


SimpleGridMapView::SimpleGridMapView(const finenav::GridMap<Voxel>& map, rclcpp::Node::SharedPtr node, std::shared_ptr<TerrainAnalyzer> terrain_analyzer)
    : map_(map), node_(node), terrain_analyzer_(terrain_analyzer) {
    if (node_) {
        local_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 10);
    }
    collision_model_ = std::make_unique<finenav::OBBCollisionModel>(finenav::Vector3D(0.3, 0.3, 0.3));
}

bool SimpleGridMapView::isTrackingUnknown() const {
    return is_tracking_unknown_;
}

bool SimpleGridMapView::considerFootprint() const {
    return consider_footprint_;
}

bool SimpleGridMapView::isCollision(float x, float y, float theta) const {
    int pose_cost = getCost(finenav::Position3D{x, y, 0.0});
    finenav::Pose pose = createPose2D(x, y, theta);
    int possible_collision_cost = ComputeCost(collision_model_->getOuterDiameter() / map_.getResolution());
    if (pose_cost == 254) {
        return true;
    }
    if (pose_cost >= possible_collision_cost) {
        return collision_model_->checkCollision(pose, CollisionRule());
    }
    return false;
}

int SimpleGridMapView::getCost(const finenav::Position3D& pos) const {
    return terrain_analyzer_->getCost(pos, map_);
}

float SimpleGridMapView::getRadius() const {
    return collision_model_->getOuterDiameter();
}

float SimpleGridMapView::costAtPose(float x, float y, float theta) const {
    finenav::Pose pose = createPose2D(x, y, theta);
    return collision_model_->checkCost(pose,[this](const finenav::Position3D& pos) {
        return this->getCost(pos);
    });
}

std::string SimpleGridMapView::getBaseFrameID() const {
    return "map";
}

void SimpleGridMapView::publishLocalMap() {
    if (!node_ || !local_pub_ ) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.header.frame_id = "map";
    for (int x = map_.getMinIndex().x(); x <= map_.getMaxIndex().x(); ++x) {
        for (int y = map_.getMinIndex().y(); y < map_.getMaxIndex().y(); ++y) {
            for(int z = map_.getMinIndex().z(); z < map_.getMaxIndex().z(); ++z) {
                finenav::Position3D pos = map_.getPosition({x, y, z});
                pcl::PointXYZI point;
                point.x = pos.x();
                point.y = pos.y();
                point.z = map_.atPosition(pos).z_height;
                cloud.push_back(point);
            }
        }
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.stamp = node_->now();
    local_pub_->publish(cloud_msg);
}

std::function<bool(const finenav::Position3D&)> SimpleGridMapView::CollisionRule() const {
    return [this](const finenav::Position3D& pos) {
        int pose_cost = this->getCost(pos);
        switch (static_cast<unsigned char>(pose_cost)) {
            case 254:
                return true;
            case 255:
                return is_tracking_unknown_ ? false : true;
        }
        return false;
    };
}

int SimpleGridMapView::ComputeCost(int distance) const {
    return terrain_analyzer_->ComputeCost(distance);
}

finenav::Pose SimpleGridMapView::createPose2D(double x, double y, double theta) const{
    finenav::Pose pose = finenav::Pose::Identity();
    pose.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = Eigen::Vector3d(x, y, 0.0);
    return pose;
}
