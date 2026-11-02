#include "goal_progress.hpp"
#include <gtest/gtest.h>

using finenav::example::GoalProgress;
using finenav::example::approachReferenceSpeed;

TEST(GoalProgress, CirclingAndReturningDoNotRefillRecoveryBudget) {
    GoalProgress progress;
    ASSERT_TRUE(progress.update(0.60));
    // A circle can travel metres while remaining at the same goal distance.
    for (int i = 0; i < 100; ++i) EXPECT_FALSE(progress.update(0.60));
    EXPECT_FALSE(progress.update(0.90));  // move away during a recovery
    EXPECT_FALSE(progress.update(0.60));  // returning is not new progress
    EXPECT_TRUE(progress.update(0.48));
    EXPECT_FALSE(progress.update(0.90));
    EXPECT_FALSE(progress.update(0.48));
    EXPECT_FALSE(progress.update(0.45));
    EXPECT_TRUE(progress.update(0.35));
    progress.reset();  // a new navigation goal gets a fresh baseline
    EXPECT_TRUE(progress.update(0.60));
}

TEST(GoalApproach, ShrinksTurningRadiusNearGoalAndHonorsCruiseSpeed) {
    EXPECT_DOUBLE_EQ(approachReferenceSpeed(0.6, 3.0, 0.4, 0.05), 0.6);
    EXPECT_DOUBLE_EQ(approachReferenceSpeed(0.6, 0.6, 0.4, 0.05), 0.24);
    EXPECT_DOUBLE_EQ(approachReferenceSpeed(0.6, 0.2, 0.4, 0.05), 0.08);
    EXPECT_DOUBLE_EQ(approachReferenceSpeed(0.6, 0.01, 0.4, 0.05), 0.05);
    EXPECT_DOUBLE_EQ(approachReferenceSpeed(0.02, 0.01, 0.4, 0.05), 0.02);
}
