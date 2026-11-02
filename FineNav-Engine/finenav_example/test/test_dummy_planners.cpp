// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>

#include <gtest/gtest.h>

#include "dummy_follow_planner.hpp"
#include "dummy_line_planner.hpp"

namespace finenav::example {
namespace {

constexpr double kEps = 1e-9;

static_assert(!std::is_base_of_v<DummyLinePlanner::IWorldView, DummyFollowPlanner::IWorldView>);
static_assert(!std::is_base_of_v<DummyFollowPlanner::IWorldView, DummyLinePlanner::IWorldView>);

geometry_msgs::msg::Pose pose(double x, double y, double z = 0.0) {
    geometry_msgs::msg::Pose result;
    result.position.x = x;
    result.position.y = y;
    result.position.z = z;
    result.orientation.w = 1.0;
    return result;
}

core::RobotState robotState(double x, double y, double yaw = 0.0) {
    core::RobotState state;
    state.pose = pose(x, y);
    state.pose.orientation.z = std::sin(yaw * 0.5);
    state.pose.orientation.w = std::cos(yaw * 0.5);
    return state;
}

template <typename PlannerT>
class TestWorldView final : public PlannerT::IWorldView {
  public:
    explicit TestWorldView(bool walkable = true) : walkable_(walkable) {}

    bool isWalkable(const common::Position3D&) const override { return walkable_; }

  private:
    bool walkable_;
};

class SamplingProbeWorldView final : public DummyLinePlanner::IWorldView {
  public:
    bool isWalkable(const common::Position3D&) const override {
        ++sample_count;
        return false;
    }

    mutable size_t sample_count{0};
};

class BlockedPositionWorldView final : public DummyLinePlanner::IWorldView {
  public:
    explicit BlockedPositionWorldView(double blocked_x) : blocked_x_(blocked_x) {}

    bool isWalkable(const common::Position3D& position) const override {
        return std::abs(position.x - blocked_x_) > kEps;
    }

  private:
    double blocked_x_;
};

TEST(DummyLinePlanner, RejectsNonfiniteGoalBeforeSampling) {
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
        for (size_t component = 0; component < 7; ++component) {
            SCOPED_TRACE(component);
            DummyLinePlanner planner;
            auto goal = pose(2.5, 0.0);
            double* fields[] = {&goal.position.x,    &goal.position.y,    &goal.position.z,   &goal.orientation.x,
                                &goal.orientation.y, &goal.orientation.z, &goal.orientation.w};
            *fields[component] = invalid;
            core::InputTrajectoryT<DummyLinePlanner> reference;
            reference.poses.push_back(goal);
            const auto state = robotState(0.0, 0.0);
            const core::PlanningContext<DummyLinePlanner> context{state, reference};
            SamplingProbeWorldView world_view;

            EXPECT_FALSE(planner.plan(context, world_view).has_value());
            EXPECT_EQ(world_view.sample_count, 0u);
            if (std::isnan(invalid)) {
                EXPECT_FALSE(planner.plan(context, TestWorldView<DummyLinePlanner>{}).has_value());
            }
        }
    }
}

TEST(DummyLinePlanner, RejectsNonfiniteStartBeforeSampling) {
    for (size_t component = 0; component < 7; ++component) {
        SCOPED_TRACE(component);
        DummyLinePlanner planner;
        auto state = robotState(0.0, 0.0);
        auto& start = state.pose;
        double* fields[] = {&start.position.x,    &start.position.y,    &start.position.z,   &start.orientation.x,
                            &start.orientation.y, &start.orientation.z, &start.orientation.w};
        *fields[component] = std::numeric_limits<double>::quiet_NaN();
        core::InputTrajectoryT<DummyLinePlanner> reference;
        reference.poses.push_back(pose(2.5, 0.0));
        const core::PlanningContext<DummyLinePlanner> context{state, reference};
        SamplingProbeWorldView world_view;

        EXPECT_FALSE(planner.plan(context, world_view).has_value());
        EXPECT_EQ(world_view.sample_count, 0u);
        EXPECT_FALSE(planner.plan(context, TestWorldView<DummyLinePlanner>{}).has_value());
    }
}

TEST(DummyLinePlanner, RejectsNonfiniteResolutionBeforeSampling) {
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
        DummyLinePlanner planner;
        dummy_line_planner::Params params;
        params.path_resolution = invalid;
        planner.configure(params);
        core::InputTrajectoryT<DummyLinePlanner> reference;
        reference.poses.push_back(pose(2.5, 0.0));
        const auto state = robotState(0.0, 0.0);
        const core::PlanningContext<DummyLinePlanner> context{state, reference};
        SamplingProbeWorldView world_view;

        EXPECT_FALSE(planner.plan(context, world_view).has_value());
        EXPECT_EQ(world_view.sample_count, 0u);
        EXPECT_FALSE(planner.plan(context, TestWorldView<DummyLinePlanner>{}).has_value());
    }
}

TEST(DummyLinePlanner, RejectsOverflowedDistanceBeforeSampling) {
    DummyLinePlanner planner;
    core::InputTrajectoryT<DummyLinePlanner> reference;
    reference.poses.push_back(pose(std::numeric_limits<double>::max(), 0.0));
    const auto state = robotState(-std::numeric_limits<double>::max(), 0.0);
    const core::PlanningContext<DummyLinePlanner> context{state, reference};
    SamplingProbeWorldView world_view;

    EXPECT_FALSE(planner.plan(context, world_view).has_value());
    EXPECT_EQ(world_view.sample_count, 0u);
}

TEST(DummyLinePlanner, RejectsExcessiveSampleRequestsBeforeSampling) {
    for (const double resolution : {1e-9, std::numeric_limits<double>::denorm_min()}) {
        DummyLinePlanner planner;
        dummy_line_planner::Params params;
        params.path_resolution = resolution;
        planner.configure(params);
        core::InputTrajectoryT<DummyLinePlanner> reference;
        reference.poses.push_back(pose(2.5, 0.0));
        const auto state = robotState(0.0, 0.0);
        const core::PlanningContext<DummyLinePlanner> context{state, reference};
        SamplingProbeWorldView world_view;

        EXPECT_FALSE(planner.plan(context, world_view).has_value());
        EXPECT_EQ(world_view.sample_count, 0u);
    }
}

TEST(DummyLinePlanner, SamplesStraightPathAndKeepsExactGoal) {
    DummyLinePlanner planner;
    dummy_line_planner::Params params;
    params.path_resolution = 1.0;
    planner.configure(params);

    core::InputTrajectoryT<DummyLinePlanner> reference;
    reference.poses.push_back(pose(2.5, 0.0));
    const auto state = robotState(0.0, 0.0);
    const core::PlanningContext<DummyLinePlanner> context{state, reference};

    const auto result = planner.plan(context, TestWorldView<DummyLinePlanner>{});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->poses.size(), 4u);
    EXPECT_NEAR(result->poses[0].position.x, 0.0, kEps);
    EXPECT_NEAR(result->poses[1].position.x, 1.0, kEps);
    EXPECT_NEAR(result->poses[2].position.x, 2.0, kEps);
    EXPECT_NEAR(result->poses[3].position.x, 2.5, kEps);
}

TEST(DummyLinePlanner, CoincidentStartAndGoalProducesOnePoint) {
    DummyLinePlanner planner;
    dummy_line_planner::Params params;
    params.path_resolution = 0.2;
    planner.configure(params);

    core::InputTrajectoryT<DummyLinePlanner> reference;
    reference.poses.push_back(pose(1.0, -2.0));
    const auto state = robotState(1.0, -2.0);
    const core::PlanningContext<DummyLinePlanner> context{state, reference};

    const auto result = planner.plan(context, TestWorldView<DummyLinePlanner>{});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->poses.size(), 1u);
    EXPECT_NEAR(result->poses.front().position.x, 1.0, kEps);
    EXPECT_NEAR(result->poses.front().position.y, -2.0, kEps);
}

TEST(DummyLinePlanner, RejectsBlockedInteriorSample) {
    DummyLinePlanner planner;
    dummy_line_planner::Params params;
    params.path_resolution = 1.0;
    planner.configure(params);

    core::InputTrajectoryT<DummyLinePlanner> reference;
    reference.poses.push_back(pose(2.5, 0.0));
    const auto state = robotState(0.0, 0.0);
    const core::PlanningContext<DummyLinePlanner> context{state, reference};

    EXPECT_FALSE(planner.plan(context, BlockedPositionWorldView{1.0}).has_value());
}

TEST(DummyLinePlanner, RejectsBlockedExactGoal) {
    DummyLinePlanner planner;
    dummy_line_planner::Params params;
    params.path_resolution = 1.0;
    planner.configure(params);

    core::InputTrajectoryT<DummyLinePlanner> reference;
    reference.poses.push_back(pose(2.5, 0.0));
    const auto state = robotState(0.0, 0.0);
    const core::PlanningContext<DummyLinePlanner> context{state, reference};

    EXPECT_FALSE(planner.plan(context, BlockedPositionWorldView{2.5}).has_value());
}

TEST(DummyLinePlanner, RejectsEmptyReference) {
    DummyLinePlanner planner;
    core::InputTrajectoryT<DummyLinePlanner> reference;
    const auto state = robotState(0.0, 0.0);
    const core::PlanningContext<DummyLinePlanner> context{state, reference};

    EXPECT_FALSE(planner.plan(context, TestWorldView<DummyLinePlanner>{}).has_value());
}

TEST(DummyFollowPlanner, TracksTheFirstPointBeyondLookaheadDistance) {
    DummyFollowPlanner planner;
    dummy_follow_planner::Params params;
    params.linear_speed = 0.5;
    params.angular_speed = 1.0;
    params.lookahead_distance = 1.5;
    planner.configure(params);

    core::InputTrajectoryT<DummyFollowPlanner> reference;
    reference.poses = {pose(0.0, 0.0), pose(1.0, 0.0), pose(2.0, 0.0), pose(2.0, 1.0), pose(2.0, 2.0)};
    const auto state = robotState(0.9, 0.0);
    const core::PlanningContext<DummyFollowPlanner> context{state, reference};

    const auto result = planner.plan(context, TestWorldView<DummyFollowPlanner>{});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->twists.size(), 1u);
    const double expected_yaw = std::atan2(1.0, 1.1);
    EXPECT_NEAR(result->twists.front().angular.z, expected_yaw, kEps);
    EXPECT_NEAR(result->twists.front().linear.x, 0.5 * (1.0 - expected_yaw / 1.5), kEps);
}

TEST(DummyFollowPlanner, UsesGoalWhenRemainingPathIsShorterThanLookahead) {
    DummyFollowPlanner planner;
    dummy_follow_planner::Params params;
    params.linear_speed = 0.5;
    params.angular_speed = 1.0;
    params.lookahead_distance = 2.0;
    planner.configure(params);

    core::InputTrajectoryT<DummyFollowPlanner> reference;
    reference.poses = {pose(0.0, 0.0), pose(1.0, 0.0), pose(2.0, 0.0)};
    const auto state = robotState(1.8, -0.2);
    const core::PlanningContext<DummyFollowPlanner> context{state, reference};

    const auto result = planner.plan(context, TestWorldView<DummyFollowPlanner>{});

    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result->twists.size(), 1u);
    EXPECT_NEAR(result->twists.front().angular.z, std::numbers::pi / 4.0, kEps);
}

TEST(DummyFollowPlanner, RejectsEmptyOrBlockedReference) {
    DummyFollowPlanner planner;
    dummy_follow_planner::Params params;
    planner.configure(params);

    core::InputTrajectoryT<DummyFollowPlanner> empty_reference;
    const auto state = robotState(0.0, 0.0);
    const core::PlanningContext<DummyFollowPlanner> empty_context{state, empty_reference};
    EXPECT_FALSE(planner.plan(empty_context, TestWorldView<DummyFollowPlanner>{}).has_value());

    core::InputTrajectoryT<DummyFollowPlanner> blocked_reference;
    blocked_reference.poses = {pose(0.0, 0.0), pose(1.0, 0.0)};
    const core::PlanningContext<DummyFollowPlanner> blocked_context{state, blocked_reference};
    EXPECT_FALSE(planner.plan(blocked_context, TestWorldView<DummyFollowPlanner>{false}).has_value());
}

}  // namespace
}  // namespace finenav::example
