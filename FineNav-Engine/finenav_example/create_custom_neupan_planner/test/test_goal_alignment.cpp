#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "goal_alignment.hpp"

using finenav::example::GoalAlignment;

TEST(GoalAlignment, PositionHysteresisAllowsRotationWithoutPhaseChatter) {
    GoalAlignment controller;
    EXPECT_FALSE(controller.update(0.3));
    EXPECT_TRUE(controller.update(0.14));
    EXPECT_TRUE(controller.update(0.20));
    EXPECT_FALSE(controller.update(0.23));
    controller.reset();
    EXPECT_FALSE(controller.active);
}

TEST(GoalAlignment, TakesShortestTurnAcrossPiAndStopsWithinTolerance) {
    GoalAlignment controller;
    const double pi = std::acos(-1.0);
    const double w = controller.angularVelocity(-2.0 * pi + 0.2, 0.0, 1.0, 3.0, 0.1);
    EXPECT_GT(w, 0.0);
    EXPECT_LE(w, 0.3 + 1e-12);
    EXPECT_DOUBLE_EQ(controller.angularVelocity(0.04, 0.1, 1.0, 3.0, 0.1), 0.0);
}

TEST(GoalAlignment, RespectsSpeedAccelerationAndMinimumRequestedRate) {
    GoalAlignment controller;
    EXPECT_NEAR(controller.angularVelocity(2.0, 0.0, 1.0, 3.0, 0.1), 0.3, 1e-12);
    EXPECT_NEAR(controller.angularVelocity(2.0, 0.5, 1.0, 3.0, 0.1), 0.6, 1e-12);
    EXPECT_NEAR(controller.angularVelocity(0.06, 0.0, 1.0, 3.0, 0.1), 0.12, 1e-12);
    EXPECT_LE(std::abs(controller.angularVelocity(-2.0, 0.0, 0.2, 3.0, 0.1)), 0.2);
}

struct Point { double x, y; };

TEST(GoalAlignment, DetectsCornerSweepThatCurrentFootprintDoesNotTouch) {
    const std::vector<Point> points{{0.32, 0.32}};
    const auto clearance = [&](double w) {
        return GoalAlignment::rotationClearance(points, 0, 0, 0, w,
                                                0.7, 0.6, 0, 3.0, 0.1);
    };
    EXPECT_NEAR(clearance(0.0), 0.02, 1e-12);
    EXPECT_LT(clearance(0.6), 0.0);
}

TEST(GoalAlignment, HandlesWorldTransformAndOffsetFootprint) {
    const double pi = std::acos(-1.0);
    const std::vector<Point> points{{2.0, 3.46}};
    // Footprint center is +0.1 m along body x when wheelbase is 0.2 m.
    EXPECT_NEAR(GoalAlignment::rotationClearance(points, 2, 3, pi / 2, 0,
                                                0.7, 0.6, 0.2, 3.0, 0.1),
                0.01, 1e-12);
}
