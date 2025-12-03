// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Dense>

namespace finenav {

// TODO: 看看 nav2 怎么处理的，这些东西需要与核心库交互，不能与核心算法耦合

using Vector3D   = Eigen::Vector3d;
using Position3D = Eigen::Vector3d;
using Orientation = Eigen::Quaterniond;
using Region3D = Eigen::AlignedBox3d;
using Pose = Eigen::Isometry3d;

// TODO: Eigen 自带了 intersection(求交), contains(包含), extend(合并) 等几何算法

}