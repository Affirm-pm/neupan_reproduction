// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <cstddef>
#include <string>

#include "finenav_mppi_controller/controller.hpp"

namespace nav2_mppi_controller {
namespace {

class TestMapView final : public IMapView {
  public:
    bool isTrackingUnknown() const override { return false; }

    bool considerFootprint() const override { return false; }

    bool isCollision(float, float, float) const override { return false; }

    float getRadius() const override { return 0.2f; }

    int getCost(const Position3D&) const override {
        ++cost_queries_;
        return 0;
    }

    float costAtPose(float, float, float) const override { return 0.0f; }

    std::string getBaseFrameID() const override { return "base_link"; }

    std::size_t costQueries() const { return cost_queries_.load(); }

  private:
    mutable std::atomic<std::size_t> cost_queries_{0};
};

finenav_mppi_controller::Params makeControllerParams() {
    finenav_mppi_controller::Params params;
    params.time_steps = 8;
    params.batch_size = 32;
    params.iteration_count = 1;
    params.model_dt = 0.1;
    params.regenerate_noises = false;
    params.vx_max = 0.5;
    params.vx_min = -0.2;
    params.vy_max = 0.0;
    params.wz_max = 1.0;
    params.vx_std = 0.1;
    params.vy_std = 0.1;
    params.wz_std = 0.1;
    params.motion_model = "DiffDrive";
    params.PathAlignCritic.offset_from_furthest = 1;
    params.PathAlignCritic.trajectory_point_step = 1;
    params.PathFollowCritic.offset_from_furthest = 1;
    params.PathAngleCritic.offset_from_furthest = 1;
    params.TwirlingCritic.enabled = false;
    params.VelocityDeadbandCritic.enabled = false;
    return params;
}

MPPIController::Context makeContext(std::size_t path_size = 24) {
    MPPIController::Context context;
    context.robot_state.pose.orientation.w = 1.0;
    context.goal_pose.position.x = 2.3;
    context.goal_pose.orientation.w = 1.0;
    context.ref_traj.poses.reserve(path_size);
    for (std::size_t i = 0; i < path_size; ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = static_cast<double>(i) * 0.1;
        pose.orientation.w = 1.0;
        context.ref_traj.poses.push_back(pose);
    }
    return context;
}

void expectFiniteBoundedOutput(const finenav::OutputDataT<MPPIController>& output,
                               const finenav_mppi_controller::Params& params) {
    ASSERT_EQ(output.poses.size(), static_cast<std::size_t>(params.time_steps));
    ASSERT_EQ(output.twists.size(), static_cast<std::size_t>(params.time_steps));

    for (std::size_t i = 0; i < output.poses.size(); ++i) {
        const auto& pose = output.poses[i];
        const auto& twist = output.twists[i];
        EXPECT_TRUE(std::isfinite(pose.position.x));
        EXPECT_TRUE(std::isfinite(pose.position.y));
        EXPECT_NEAR(pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w, 1.0, 1e-5);
        EXPECT_TRUE(std::isfinite(twist.linear.x));
        EXPECT_GE(twist.linear.x, params.vx_min - 1e-5);
        EXPECT_LE(twist.linear.x, params.vx_max + 1e-5);
        EXPECT_NEAR(twist.linear.y, 0.0, 1e-6);
        EXPECT_LE(std::abs(twist.angular.z), params.wz_max + 1e-5);
    }
}

TEST(ControllerPlanTest, PlanReturnsFiniteBoundedTrajectory) {
    const auto params = makeControllerParams();
    MPPIController controller;
    controller.configure(params);
    TestMapView map_view;

    const auto result = controller.plan(makeContext(), map_view);

    ASSERT_TRUE(result.has_value());
    expectFiniteBoundedOutput(*result, params);
    EXPECT_GT(map_view.costQueries(), 0u);
}

TEST(ControllerPlanTest, ResetAllowsPlanningAgain) {
    const auto params = makeControllerParams();
    MPPIController controller;
    controller.configure(params);
    TestMapView map_view;

    ASSERT_TRUE(controller.plan(makeContext(), map_view).has_value());
    controller.reset();
    const auto second = controller.plan(makeContext(), map_view);

    ASSERT_TRUE(second.has_value());
    expectFiniteBoundedOutput(*second, params);
}

TEST(ControllerPlanTest, PlanPreservesConfiguredHorizon) {
    auto params = makeControllerParams();
    params.time_steps = 5;
    MPPIController controller;
    controller.configure(params);
    TestMapView map_view;

    const auto result = controller.plan(makeContext(), map_view);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->poses.size(), 5u);
    EXPECT_EQ(result->twists.size(), 5u);
}

TEST(ControllerPlanTest, EmptyReferencePathReturnsNoPlan) {
    const auto params = makeControllerParams();
    MPPIController controller;
    controller.configure(params);
    TestMapView map_view;
    const auto context = makeContext(0);

    const auto result = controller.plan(context, map_view);

    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(map_view.costQueries(), 0u);
}

}  // namespace
}  // namespace nav2_mppi_controller
