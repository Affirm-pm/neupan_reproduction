// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "finenav_interfaces/msg/robot_state.hpp"

namespace finenav::core {

inline void fromMsg(const finenav_interfaces::msg::RobotState& msg, Eigen::Vector3d& pos) {
    pos.x() = msg.pose.position.x;
    pos.y() = msg.pose.position.y;
    pos.z() = msg.pose.position.z;
}

inline void toMsg(const Eigen::Vector3d& pos, finenav_interfaces::msg::RobotState& msg) {
    msg.pose.position.x = pos.x();
    msg.pose.position.y = pos.y();
    msg.pose.position.z = pos.z();
}

inline void fromMsg(const finenav_interfaces::msg::RobotState& msg, Eigen::Quaterniond& ori) {
    ori = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x,
                              msg.pose.orientation.y, msg.pose.orientation.z);
}

inline void toMsg(const Eigen::Quaterniond& ori, finenav_interfaces::msg::RobotState& msg) {
    msg.pose.orientation.x = ori.x();
    msg.pose.orientation.y = ori.y();
    msg.pose.orientation.z = ori.z();
    msg.pose.orientation.w = ori.w();
}

inline void fromMsgLinearVel(const finenav_interfaces::msg::RobotState& msg, Eigen::Vector3d& v) {
    v.x() = msg.twist.linear.x;
    v.y() = msg.twist.linear.y;
    v.z() = msg.twist.linear.z;
}

inline void toMsgLinearVel(const Eigen::Vector3d& v, finenav_interfaces::msg::RobotState& msg) {
    msg.twist.linear.x = v.x();
    msg.twist.linear.y = v.y();
    msg.twist.linear.z = v.z();
}

inline void fromMsgAngularVel(const finenav_interfaces::msg::RobotState& msg, Eigen::Vector3d& w) {
    w.x() = msg.twist.angular.x;
    w.y() = msg.twist.angular.y;
    w.z() = msg.twist.angular.z;
}

inline void toMsgAngularVel(const Eigen::Vector3d& w, finenav_interfaces::msg::RobotState& msg) {
    msg.twist.angular.x = w.x();
    msg.twist.angular.y = w.y();
    msg.twist.angular.z = w.z();
}

inline void fromMsg(const finenav_interfaces::msg::RobotState& msg,
                    Eigen::Vector3d& pos, Eigen::Quaterniond& ori,
                    Eigen::Vector3d& lin_vel, Eigen::Vector3d& ang_vel) {
    fromMsg(msg, pos);
    fromMsg(msg, ori);
    fromMsgLinearVel(msg, lin_vel);
    fromMsgAngularVel(msg, ang_vel);
}

inline void toMsg(const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori,
                  const Eigen::Vector3d& lin_vel, const Eigen::Vector3d& ang_vel,
                  finenav_interfaces::msg::RobotState& msg) {
    toMsg(pos, msg);
    toMsg(ori, msg);
    toMsgLinearVel(lin_vel, msg);
    toMsgAngularVel(ang_vel, msg);
}

}  // namespace finenav::core
