// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV_PLANNER__FINENAV_MPPI_CONTROLLER__BENCHMARK__COST_CRITIC_HEAVY_FIXTURE_HPP_
#define FINENAV_PLANNER__FINENAV_MPPI_CONTROLLER__BENCHMARK__COST_CRITIC_HEAVY_FIXTURE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "finenav_collision_model/obb_collision_model.hpp"
#include "finenav_mppi_controller/map_view_def.hpp"
#include "finenav_mppi_controller/models/trajectories.hpp"

namespace mppi::benchmark_support
{

class HeavyObstacleMapView final : public nav2_mppi_controller::IMapView
{
public:
  HeavyObstacleMapView()
  : collision_model_(
      finenav::Vector3D(0.8, 0.7, 0.3), finenav::Pose::Identity(),
      finenav::OBBCollisionPolicy{.step = kResolution}),
    costs_(kGridSize * kGridSize, kInflatedCost)
  {
    // The walls are one cell wide. A trajectory first finds the wall through
    // its OBB edge before the center point reaches the occupied cell.
    fillVerticalWall(0.70F, 0.0F);
    fillVerticalWall(1.10F, 1.0F);
    fillVerticalWall(1.55F, 2.0F);
  }

  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return true;}
  float getRadius() const override {return static_cast<float>(collision_model_.getOuterDiameter());}
  std::string getBaseFrameID() const override {return "base_link";}

  int getCost(const Position3D & position) const override
  {
    const int x_index = worldToGridIndex(static_cast<float>(position.x()));
    const int y_index = worldToGridIndex(static_cast<float>(position.y()));
    if (x_index < 0 || y_index < 0 || x_index >= kGridSize || y_index >= kGridSize) {
      return kCollisionCost;
    }
    return costs_[static_cast<std::size_t>(y_index) * kGridSize + x_index];
  }

  float costAtPose(float x, float y, float) const override
  {
    return static_cast<float>(getCost(Position3D{x, y, 0.0}));
  }

  bool isCollision(float x, float y, float theta) const override
  {
    if (getCost(Position3D{x, y, 0.0}) == kCollisionCost) {
      return true;
    }

    if (getCost(Position3D{x, y, 0.0}) < kFootprintCheckCost) {
      return false;
    }

    finenav::Pose pose = finenav::Pose::Identity();
    pose.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pose.translation() = finenav::Vector3D(x, y, 0.0);
    return collision_model_.checkCollisionEdgeStateless(
      pose, [this](const finenav::Position3D & position) {
        return getCost(position) == kCollisionCost;
      });
  }

private:
  static constexpr float kResolution = 0.025F;
  static constexpr float kOrigin = -16.0F;
  static constexpr int kGridSize = 1280;
  static constexpr int kInflatedCost = 200;
  static constexpr int kCollisionCost = 254;
  static constexpr int kFootprintCheckCost = 192;

  int worldToGridIndex(float coordinate) const
  {
    return static_cast<int>(std::floor((coordinate - kOrigin) / kResolution));
  }

  void fillVerticalWall(float x, float y_center)
  {
    const int x_index = worldToGridIndex(x);
    const int y_begin = worldToGridIndex(y_center - 0.5F);
    const int y_end = worldToGridIndex(y_center + 0.5F);
    for (int y_index = std::max(0, y_begin); y_index <= std::min(kGridSize - 1, y_end); ++y_index) {
      costs_[static_cast<std::size_t>(y_index) * kGridSize + x_index] = kCollisionCost;
    }
  }

  finenav::OBBCollisionModel collision_model_;
  std::vector<std::uint8_t> costs_;
};

inline void populateHeavyObstacleTrajectories(models::Trajectories & trajectories)
{
  const Eigen::Index trajectory_count = trajectories.x.rows();
  for (Eigen::Index trajectory = 0; trajectory < trajectories.x.rows(); ++trajectory) {
    // Keep similarly expensive trajectories adjacent so static scheduling has
    // the same sustained imbalance as obstacle-correlated MPPI samples.
    const unsigned int group = static_cast<unsigned int>(
      trajectory * 20U / trajectory_count);
    float lateral = -2.5F;
    if (group >= 8U && group < 12U) {
      lateral = 0.0F;
    } else if (group >= 12U && group < 16U) {
      lateral = 1.0F;
    } else if (group >= 16U && group < 19U) {
      lateral = 2.0F;
    } else if (group == 19U) {
      lateral = 3.0F;
    }

    for (Eigen::Index step = 0; step < trajectories.x.cols(); ++step) {
      trajectories.x(trajectory, step) = 0.06F * static_cast<float>(step);
      trajectories.y(trajectory, step) = lateral;
      trajectories.yaws(trajectory, step) = 0.01F * static_cast<float>(trajectory % 13);
    }
  }
}

}  // namespace mppi::benchmark_support

#endif  // FINENAV_PLANNER__FINENAV_MPPI_CONTROLLER__BENCHMARK__COST_CRITIC_HEAVY_FIXTURE_HPP_
