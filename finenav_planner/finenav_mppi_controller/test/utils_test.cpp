// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <cmath>

#include "finenav_mppi_controller/models/eigen_types.hpp"
#include "finenav_mppi_controller/tools/utils.hpp"

namespace mppi::utils
{
namespace
{

template<typename T>
constexpr T degreesToRadians(T degrees)
{
  return degrees * static_cast<T>(M_PI) / static_cast<T>(180.0);
}

TEST(UtilsTest, NormalizeAnglesSupportsScalarFloat)
{
  EXPECT_NEAR(
    normalize_angles(degreesToRadians(370.0F)),
    degreesToRadians(10.0F), 1.0e-6F);
}

TEST(UtilsTest, NormalizeAnglesSupportsScalarDouble)
{
  EXPECT_NEAR(
    normalize_angles(degreesToRadians(-725.0)),
    degreesToRadians(-5.0), 1.0e-12);
}

TEST(UtilsTest, ShortestAngularDistanceWrapsScalarsAcrossPi)
{
  EXPECT_NEAR(
    shortest_angular_distance(degreesToRadians(170.0F), degreesToRadians(-170.0F)),
    degreesToRadians(20.0F), 1.0e-6F);
  EXPECT_NEAR(
    shortest_angular_distance(degreesToRadians(-170.0F), degreesToRadians(170.0F)),
    degreesToRadians(-20.0F), 1.0e-6F);
}

TEST(UtilsTest, NormalizeAnglesPreservesEigenBatchBehavior)
{
  models::Vector angles(3);
  angles << degreesToRadians(370.0F), degreesToRadians(-370.0F),
    degreesToRadians(180.0F);

  const auto normalized = normalize_angles(angles);

  EXPECT_NEAR(normalized(0), degreesToRadians(10.0F), 1.0e-6F);
  EXPECT_NEAR(normalized(1), degreesToRadians(-10.0F), 1.0e-6F);
  EXPECT_NEAR(normalized(2), degreesToRadians(180.0F), 1.0e-6F);
}

}  // namespace
}  // namespace mppi::utils
