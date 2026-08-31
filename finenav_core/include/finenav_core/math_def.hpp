// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <Eigen/Dense>

namespace finenav::core {

// TODO：取缔。不做math_def，公用def目前只需要做 坐标系  WORLD_FIXED_FRAME, ROBOT_FRAME
/**
 * @brief Math definition that used in FineNav-Engine
 */
using Position3D = Eigen::Vector3d;
using Region3D = Eigen::AlignedBox3d;


}