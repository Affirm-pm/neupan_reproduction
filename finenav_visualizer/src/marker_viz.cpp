// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/marker_viz.hpp"

#include <cmath>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace finenav::viz {

namespace {

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& v) {
    geometry_msgs::msg::Point p;
    p.x = v.x(); p.y = v.y(); p.z = v.z();
    return p;
}

geometry_msgs::msg::Pose toPose(const Eigen::Isometry3d& T) {
    geometry_msgs::msg::Pose p;
    p.position.x = T.translation().x();
    p.position.y = T.translation().y();
    p.position.z = T.translation().z();
    const Eigen::Quaterniond q(T.rotation());
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    return p;
}

geometry_msgs::msg::Vector3 toVector3(double x, double y, double z) {
    geometry_msgs::msg::Vector3 v;
    v.x = x; v.y = y; v.z = z;
    return v;
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

}  // namespace

MarkerViz::MarkerViz(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {}

std::int32_t MarkerViz::allocateId()
{
    return next_marker_id_++;
}

void MarkerViz::setLifetime(double sec) {
    for (auto& m : markers_.markers) {
        m.lifetime.sec = static_cast<int32_t>(sec);
        m.lifetime.nanosec = static_cast<uint32_t>((sec - std::floor(sec)) * 1e9);
    }
}

void MarkerViz::addSphere(const Eigen::Vector3d& center, double radius,
                               const std_msgs::msg::ColorRGBA& color) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.ns = "Sphere";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = toPose(Eigen::Translation3d(center) * Eigen::Quaterniond::Identity());
    m.scale = toVector3(radius * 2, radius * 2, radius * 2);
    m.color = color;
    m.color.a *= alpha_;
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addSpheres(const std::vector<Eigen::Vector3d>& centers,
                                double radius,
                                const std::vector<std_msgs::msg::ColorRGBA>& colors) {
    for (size_t i = 0; i < centers.size(); ++i) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = base_frame_;
        m.ns = "Spheres";
        m.id = allocateId();
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose = toPose(Eigen::Translation3d(centers[i]) * Eigen::Quaterniond::Identity());
        m.scale = toVector3(radius * 2, radius * 2, radius * 2);
        m.color = (i < colors.size()) ? colors[i] : makeColor(0.5f, 0.5f, 0.5f, 1.0f);
        m.color.a *= alpha_;
        markers_.markers.push_back(std::move(m));
    }
}

void MarkerViz::addArrow(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                              const std_msgs::msg::ColorRGBA& color,
                              double shaft_d, double head_d, double head_len) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.ns = "Arrow";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.points.push_back(toPoint(start));
    m.points.push_back(toPoint(end));
    m.scale = toVector3(shaft_d, head_d, head_len);
    m.color = color;
    m.color.a *= alpha_;
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addCuboid(const Eigen::Isometry3d& pose, double depth, double width, double height,
                               const std_msgs::msg::ColorRGBA& color) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.ns = "Cuboid";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = toPose(pose);
    m.scale = toVector3(depth, width, height);
    m.color = color;
    m.color.a *= alpha_;
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addWireframeCuboid(const Eigen::Isometry3d& pose,
                                        double depth, double width, double height,
                                        const std_msgs::msg::ColorRGBA& color) {
    const double hd = depth / 2.0, hw = width / 2.0, hh = height / 2.0;
    const Eigen::Vector3d corners[8] = {
        pose * Eigen::Vector3d(-hd, -hw, -hh), pose * Eigen::Vector3d( hd, -hw, -hh),
        pose * Eigen::Vector3d( hd,  hw, -hh), pose * Eigen::Vector3d(-hd,  hw, -hh),
        pose * Eigen::Vector3d(-hd, -hw,  hh), pose * Eigen::Vector3d( hd, -hw,  hh),
        pose * Eigen::Vector3d( hd,  hw,  hh), pose * Eigen::Vector3d(-hd,  hw,  hh),
    };
    const int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}
    };
    for (auto& e : edges) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = base_frame_;
        m.ns = "Wireframe";
        m.id = allocateId();
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.02;
        m.points.push_back(toPoint(corners[e[0]]));
        m.points.push_back(toPoint(corners[e[1]]));
        m.color = color;
        m.color.a *= alpha_;
        markers_.markers.push_back(std::move(m));
    }
}

void MarkerViz::addCone(const Eigen::Isometry3d& pose, double angle, double max_range,
                             const std_msgs::msg::ColorRGBA& color) {
    const double r = std::tan(angle) * max_range;
    constexpr int N = 32;
    for (int i = 0; i < N; ++i) {
        const double a0 = 2.0 * M_PI * i / N;
        const double a1 = 2.0 * M_PI * (i + 1) / N;
        const Eigen::Vector3d tip = pose.translation();
        const Eigen::Vector3d p0  = pose * Eigen::Vector3d(r * std::cos(a0), r * std::sin(a0), max_range);
        const Eigen::Vector3d p1  = pose * Eigen::Vector3d(r * std::cos(a1), r * std::sin(a1), max_range);

        visualization_msgs::msg::Marker m;
        m.header.frame_id = base_frame_;
        m.ns = "FOV";
        m.id = allocateId();
        m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.points.push_back(toPoint(tip));
        m.points.push_back(toPoint(p0));
        m.points.push_back(toPoint(p1));
        m.scale = toVector3(1, 1, 1);
        m.color = color;
        m.color.a *= alpha_;
        markers_.markers.push_back(std::move(m));
    }
}

void MarkerViz::addCylinder(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                 double radius, const std_msgs::msg::ColorRGBA& color) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.ns = "Cylinder";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;

    const Eigen::Vector3d mid = (p1 + p2) * 0.5;
    const Eigen::Vector3d dir = (p2 - p1).normalized();
    const double len = (p2 - p1).norm();

    const Eigen::Vector3d z(0, 0, 1);
    const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z, dir);

    m.pose = toPose(Eigen::Translation3d(mid) * q);
    m.scale = toVector3(radius * 2, radius * 2, len);
    m.color = color;
    m.color.a *= alpha_;
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                             const std_msgs::msg::ColorRGBA& color, double width) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.ns = "Line";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = width;
    m.points.push_back(toPoint(p1));
    m.points.push_back(toPoint(p2));
    m.color = color;
    m.color.a *= alpha_;
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addLineStrip(const std::vector<Eigen::Vector3d>& points,
                             const std_msgs::msg::ColorRGBA& color,
                             double width,
                             const std::string& ns) {
    if (points.empty()) return;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.ns = ns;
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = width;
    m.color = color;
    m.color.a *= alpha_;
    m.points.reserve(points.size());
    for (const auto& point : points) {
        m.points.push_back(toPoint(point));
    }
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addPath(const std::vector<Eigen::Vector3d>& points,
                             const std_msgs::msg::ColorRGBA& color, double radius) {
    if (points.size() < 2) return;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        addCylinder(points[i], points[i + 1], radius, color);
    }
}

void MarkerViz::addAxis(const Eigen::Isometry3d& pose, double length, double radius) {
    const Eigen::Vector3d o = pose.translation();
    const Eigen::Vector3d x = o + pose.rotation() * Eigen::Vector3d(length, 0, 0);
    const Eigen::Vector3d y = o + pose.rotation() * Eigen::Vector3d(0, length, 0);
    const Eigen::Vector3d z = o + pose.rotation() * Eigen::Vector3d(0, 0, length);
    addArrow(o, x, makeColor(1, 0.1f, 0.1f, alpha_), radius, radius * 3, length * 0.3f);
    addArrow(o, y, makeColor(0.1f, 1, 0.1f, alpha_), radius, radius * 3, length * 0.3f);
    addArrow(o, z, makeColor(0.1f, 0.3f, 1, alpha_), radius, radius * 3, length * 0.3f);
}

void MarkerViz::addText(const Eigen::Vector3d& pos, const std::string& text,
                             const std_msgs::msg::ColorRGBA& color, double height) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = base_frame_;
    m.ns = "Text";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = toPose(Eigen::Translation3d(pos) * Eigen::Quaterniond::Identity());
    m.scale.z = height;
    m.text = text;
    m.color = color;
    m.color.a *= alpha_;
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::publish(const std::string& topic) {
    if (markers_.markers.empty()) return;

    const auto now = node_->get_clock()->now();
    for (auto& m : markers_.markers) {
        m.header.stamp = now;
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub;
    {
        std::shared_lock lock(pubs_mutex_);
        auto it = pubs_.find(topic);
        if (it != pubs_.end()) pub = it->second;
    }
    if (!pub) {
        std::unique_lock lock(pubs_mutex_);
        auto& p = pubs_[topic];
        if (!p) p = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
        pub = p;
    }
    pub->publish(markers_);
    clear();
}

void MarkerViz::deleteAllMarkers(const std::string& topic) {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub;
    {
        std::shared_lock lock(pubs_mutex_);
        auto it = pubs_.find(topic);
        if (it != pubs_.end()) pub = it->second;
    }
    if (!pub) {
        std::unique_lock lock(pubs_mutex_);
        auto& p = pubs_[topic];
        if (!p) p = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
        pub = p;
    }
    pub->publish(arr);
    clear();
}

bool MarkerViz::hasSubscribers(const std::string& topic) const {
    std::shared_lock lock(pubs_mutex_);
    auto it = pubs_.find(topic);
    if (it == pubs_.end()) return false;
    return it->second->get_subscription_count() > 0;
}

}  // namespace finenav::viz
