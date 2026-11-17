#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "global_grid_world_view.hpp"

namespace finenav::example {
struct GlobalGridWorldViewTestAccess {
    static void set(GlobalGridWorldView& view, nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
        std::lock_guard lock(view.mutex_);
        view.pending_ = std::move(map);
    }
};
namespace {
geometry_msgs::msg::Pose pose(double x, double y) {
    geometry_msgs::msg::Pose p;
    p.position.x = x; p.position.y = y; p.orientation.w = 1;
    return p;
}
class GlobalPlannerTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { if (!rclcpp::ok()) rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
        "global_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    DummyMap dummy;
    GlobalGridWorldView view{node, dummy};
    GlobalPlanner planner;
    core::RobotState state;
    core::InputTrajectoryT<GlobalPlanner> reference;
    nav_msgs::msg::OccupancyGrid::SharedPtr map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    void SetUp() override {
        map->header.frame_id = "map";
        map->info.width = map->info.height = 100;
        map->info.resolution = 0.1;
        map->info.origin.orientation.w = 1;
        map->data.resize(10000, 0);
        state.pose = pose(2, 4);
        reference.poses.push_back(pose(8, 4));
        reference.poses.back().orientation.z = std::sin(0.7);
        reference.poses.back().orientation.w = std::cos(0.7);
        global_grid_world_view::Params view_config;
        finenav_global_planner::Params planner_config;
        view.configure(view_config);
        planner.configure(planner_config);
    }
    void prepare() {
        GlobalGridWorldViewTestAccess::set(view, map);
        view.prepare({state, reference});
    }
};
TEST_F(GlobalPlannerTest, DetoursAroundWallAndPreservesGoalYaw) {
    for (int y = 0; y < 65; ++y) for (int x = 48; x <= 52; ++x) map->data[y*100+x] = 100;
    prepare();
    auto result = planner.plan({state, reference}, view);
    ASSERT_TRUE(result);
    double ymax = 0;
    for (const auto& p : result->poses) {
        ymax = std::max(ymax, p.position.y);
        EXPECT_TRUE(view.isWalkable({p.position.x, p.position.y, p.position.z}));
    }
    EXPECT_GT(ymax, 7.0);
    EXPECT_EQ(result->poses.back(), reference.poses.back());
    EXPECT_EQ(result->poses.front(), state.pose);
}
TEST_F(GlobalPlannerTest, RejectsSealedWallAndBlockedGoal) {
    for (int y = 0; y < 100; ++y) map->data[y*100+50] = 100;
    prepare();
    EXPECT_FALSE(planner.plan({state, reference}, view));
    reference.poses.back() = pose(5, 5);
    EXPECT_FALSE(planner.plan({state, reference}, view));
}
TEST_F(GlobalPlannerTest, RejectsMissingMalformedAndWrongFrameMaps) {
    view.prepare({state, reference});
    EXPECT_FALSE(planner.plan({state, reference}, view));
    for (int fault = 0; fault < 5; ++fault) {
        auto saved = map;
        map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*saved);
        if (fault == 0) map->header.frame_id = "odom";
        if (fault == 1) map->data.pop_back();
        if (fault == 2) map->info.resolution = 0;
        if (fault == 3) map->info.origin.orientation.z = 0.2;
        if (fault == 4) map->info.origin.position.x = std::numeric_limits<double>::quiet_NaN();
        prepare();
        EXPECT_FALSE(planner.plan({state, reference}, view));
        map = saved;
    }
}
TEST_F(GlobalPlannerTest, UnknownCellsInflationAndMapReplacement) {
    map->data[50*100+50] = -1;
    prepare();
    EXPECT_FALSE(view.isWalkable({5.05, 5.05, 0}));
    EXPECT_FALSE(view.isWalkable({5.45, 5.05, 0}));
    EXPECT_FALSE(view.isWalkable({0.1, 5, 0}));
    EXPECT_FALSE(view.isWalkable({-1, 5, 0}));
    EXPECT_TRUE(view.isWalkable({6, 5, 0}));
    map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
    map->data[50*100+50] = 0;
    prepare();
    EXPECT_TRUE(view.isWalkable({5.05, 5.05, 0}));
}
TEST_F(GlobalPlannerTest, ExactEndpointInSameCellAndInvalidInput) {
    state.pose = pose(2.02, 2.02);
    reference.poses.back() = pose(2.08, 2.08);
    prepare();
    auto result = planner.plan({state, reference}, view);
    ASSERT_TRUE(result);
    EXPECT_EQ(result->poses.back(), reference.poses.back());
    reference.poses.back().position.x = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(planner.plan({state, reference}, view));
    reference.poses.clear();
    EXPECT_FALSE(planner.plan({state, reference}, view));
}
TEST_F(GlobalPlannerTest, NonalignedOriginStillUsesCorrectCoordinates) {
    map->info.origin.position.x = -0.037;
    map->info.origin.position.y = -0.083;
    prepare();
    EXPECT_TRUE(planner.plan({state, reference}, view));
}
TEST_F(GlobalPlannerTest, UpdatesStartAndRejectsNewlyBlockedStart) {
    prepare();
    ASSERT_TRUE(planner.plan({state, reference}, view));
    state.pose = pose(4, 4.35);
    const auto route = planner.plan({state, reference}, view);
    ASSERT_TRUE(route);
    EXPECT_EQ(route->poses.front(), state.pose);
    map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map);
    map->data[49*100+40] = 100;
    prepare();
    EXPECT_FALSE(planner.plan({state, reference}, view));
    EXPECT_GT(view.getCost({4.05, 4.05, 0}), view.getCost({4.05, 3.05, 0}));
}
TEST_F(GlobalPlannerTest, ExactGoalDoesNotCreateAReversedTerminalSegment) {
    state.pose = pose(2,2);
    reference.poses.back() = pose(2,3);
    prepare();
    const auto route = planner.plan({state, reference}, view);
    ASSERT_TRUE(route);
    double previous_y = state.pose.position.y;
    for (const auto& p : route->poses) {
        EXPECT_GE(p.position.y, previous_y);
        EXPECT_LE(p.position.y, 3.0);
        previous_y = p.position.y;
    }
}
}  // namespace
}  // namespace finenav::example
