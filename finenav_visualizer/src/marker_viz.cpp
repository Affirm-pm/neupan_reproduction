// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/marker_viz.hpp"

#include <cmath>
#include <mutex>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>

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

/// The one place a Color becomes a ROS colour message.  Nothing converts on the
/// way: the message's channels are float and so are Color's — that is why they
/// are — so what a viewer receives is bit-for-bit what was staged.
///
/// It stays in this translation unit so no public header has to name std_msgs:
/// the message type is how a marker is encoded, not how a colour is spelled.
std_msgs::msg::ColorRGBA toColorRGBA(const Color& color) {
    std_msgs::msg::ColorRGBA c;
    c.r = color.r;
    c.g = color.g;
    c.b = color.b;
    c.a = color.a;
    return c;
}

}  // namespace

std::int32_t MarkerViz::allocateId()
{
    return next_marker_id_++;
}

void MarkerViz::addSphere(const Eigen::Vector3d& center, double radius,
                               const Color& color) {
    std::lock_guard<std::mutex> lock(mutex_);
    visualization_msgs::msg::Marker m;
    m.ns = "Sphere";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = toPose(Eigen::Translation3d(center) * Eigen::Quaterniond::Identity());
    m.scale = toVector3(radius * 2, radius * 2, radius * 2);
    m.color = toColorRGBA(color);
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addArrow(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                         double shaft_d, double head_d, double head_len,
                         const Color& color) {
    std::lock_guard<std::mutex> lock(mutex_);
    addArrowUnlocked(start, end, shaft_d, head_d, head_len, color);
}

void MarkerViz::addArrowUnlocked(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                                 double shaft_d, double head_d, double head_len,
                                 const Color& color) {
    visualization_msgs::msg::Marker m;
    m.ns = "Arrow";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.points.push_back(toPoint(start));
    m.points.push_back(toPoint(end));
    m.scale = toVector3(shaft_d, head_d, head_len);
    m.color = toColorRGBA(color);
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addCuboid(const Eigen::Isometry3d& pose, double depth, double width, double height,
                               const Color& color) {
    std::lock_guard<std::mutex> lock(mutex_);
    visualization_msgs::msg::Marker m;
    m.ns = "Cuboid";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = toPose(pose);
    m.scale = toVector3(depth, width, height);
    m.color = toColorRGBA(color);
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addWireframeCuboid(const Eigen::Isometry3d& pose,
                                        double depth, double width, double height,
                                        const Color& color) {
    std::lock_guard<std::mutex> lock(mutex_);
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
        m.ns = "Wireframe";
        m.id = allocateId();
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.02;
        m.points.push_back(toPoint(corners[e[0]]));
        m.points.push_back(toPoint(corners[e[1]]));
        m.color = toColorRGBA(color);
        markers_.markers.push_back(std::move(m));
    }
}

void MarkerViz::addCone(const Eigen::Isometry3d& pose, double angle, double max_range,
                             const Color& color) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double r = std::tan(angle) * max_range;
    constexpr int N = 32;
    for (int i = 0; i < N; ++i) {
        const double a0 = 2.0 * M_PI * i / N;
        const double a1 = 2.0 * M_PI * (i + 1) / N;
        const Eigen::Vector3d tip = pose.translation();
        const Eigen::Vector3d p0  = pose * Eigen::Vector3d(r * std::cos(a0), r * std::sin(a0), max_range);
        const Eigen::Vector3d p1  = pose * Eigen::Vector3d(r * std::cos(a1), r * std::sin(a1), max_range);

        visualization_msgs::msg::Marker m;
        m.ns = "FOV";
        m.id = allocateId();
        m.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.points.push_back(toPoint(tip));
        m.points.push_back(toPoint(p0));
        m.points.push_back(toPoint(p1));
        m.scale = toVector3(1, 1, 1);
        m.color = toColorRGBA(color);
        markers_.markers.push_back(std::move(m));
    }
}

void MarkerViz::addCylinder(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                 double radius, const Color& color) {
    std::lock_guard<std::mutex> lock(mutex_);
    addCylinderUnlocked(p1, p2, radius, color);
}

void MarkerViz::addCylinderUnlocked(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                    double radius, const Color& color) {
    visualization_msgs::msg::Marker m;
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
    m.color = toColorRGBA(color);
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                        double width, const Color& color) {
    std::lock_guard<std::mutex> lock(mutex_);
    visualization_msgs::msg::Marker m;
    m.ns = "Line";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = width;
    m.points.push_back(toPoint(p1));
    m.points.push_back(toPoint(p2));
    m.color = toColorRGBA(color);
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addLineStrip(const std::vector<Eigen::Vector3d>& points,
                             double width, const Color& color) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (points.empty()) return;

    visualization_msgs::msg::Marker m;
    m.ns = "LineStrip";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = width;
    m.color = toColorRGBA(color);
    m.points.reserve(points.size());
    for (const auto& point : points) {
        m.points.push_back(toPoint(point));
    }
    markers_.markers.push_back(std::move(m));
}

void MarkerViz::addPath(const std::vector<Eigen::Vector3d>& points,
                        double radius, const Color& color) {
    // One path, one lock: the cylinders below are the segments of a single
    // shape, and the *Unlocked helper is the same code addCylinder() runs.
    std::lock_guard<std::mutex> lock(mutex_);
    if (points.size() < 2) return;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        addCylinderUnlocked(points[i], points[i + 1], radius, color);
    }
}

void MarkerViz::addAxis(const Eigen::Isometry3d& pose, double length, double radius) {
    // Three arrows are one axis to whoever asked for it, so the lock spans all
    // three: another thread's shapes cannot land between the X and the Z.
    std::lock_guard<std::mutex> lock(mutex_);
    const Eigen::Vector3d o = pose.translation();
    const Eigen::Vector3d x = o + pose.rotation() * Eigen::Vector3d(length, 0, 0);
    const Eigen::Vector3d y = o + pose.rotation() * Eigen::Vector3d(0, length, 0);
    const Eigen::Vector3d z = o + pose.rotation() * Eigen::Vector3d(0, 0, length);
    addArrowUnlocked(o, x, radius, radius * 3, length * 0.3f, Color{1.0f, 0.1f, 0.1f});
    addArrowUnlocked(o, y, radius, radius * 3, length * 0.3f, Color{0.1f, 1.0f, 0.1f});
    addArrowUnlocked(o, z, radius, radius * 3, length * 0.3f, Color{0.1f, 0.3f, 1.0f});
}

void MarkerViz::addText(const Eigen::Vector3d& pos, const std::string& text,
                        double height, const Color& color) {
    std::lock_guard<std::mutex> lock(mutex_);
    visualization_msgs::msg::Marker m;
    m.ns = "Text";
    m.id = allocateId();
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = toPose(Eigen::Translation3d(pos) * Eigen::Quaterniond::Identity());
    m.scale.z = height;
    m.text = text;
    m.color = toColorRGBA(color);
    markers_.markers.push_back(std::move(m));
}

visualization_msgs::msg::MarkerArray MarkerViz::takeMessage(common::Frame frame,
                                                            const rclcpp::Time& stamp) {
    // A message is the whole picture, never a patch on the last one: a shape
    // the algorithm stopped staging has to vanish, and only this can make it.
    //
    // The namespace is left EMPTY on purpose.  A DELETEALL carrying a namespace
    // clears that namespace alone, and every shape here is stamped with one
    // (Sphere, Arrow, …) — so a namespaced reset would clear nothing at all.
    // Empty is the display-wide delete, which is what "this is everything"
    // means.  It goes first because the viewer consumes the array in order.
    visualization_msgs::msg::Marker reset;
    reset.action = visualization_msgs::msg::Marker::DELETEALL;
    markers_.markers.insert(markers_.markers.begin(), std::move(reset));

    // Stamped in place — and that is the whole point of taking rather than
    // building: the array moves out below with its point vectors intact, so a
    // publish of N markers costs one pass over N headers, not N deep copies.
    // The tag is turned into a name once, here: every marker in the message
    // shares it, and resolving it per marker would read the frame names N times
    // for one answer.
    const std::string& frame_id = common::frameName(frame);
    for (auto& m : markers_.markers) {
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
    }

    next_marker_id_ = 0;  // handed over: the next cycle numbers its shapes afresh
    return std::move(markers_);
}

}  // namespace finenav::viz
