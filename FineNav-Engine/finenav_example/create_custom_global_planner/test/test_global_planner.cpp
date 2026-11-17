#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <thread>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "global_grid_world_view.hpp"

namespace finenav::example {
struct NeuPanWorldViewTestAccess {
    static void set(NeuPanWorldView& view, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        std::lock_guard lock(view.mutex_);
        view.cloud_ = std::move(cloud);
        view.received_ = std::chrono::steady_clock::now();
    }
};
struct GlobalGridWorldViewTestAccess {
    static void set(GlobalGridWorldView& view, nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
        std::lock_guard lock(view.mutex_);
        view.pending_ = std::move(map);
    }
    static void cloud(GlobalGridWorldView& view, sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        NeuPanWorldViewTestAccess::set(view.observations_, std::move(cloud));
    }
    static void expireCells(GlobalGridWorldView& view) {
        for (auto& [cell, hit] : view.live_cells_) hit.stamp -= std::chrono::seconds(4);
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
    void enableLiveObstacles() {
        global_grid_world_view::Params cfg;
        cfg.use_live_obstacles = true;
        view.configure(cfg);
        node->set_parameters({rclcpp::Parameter("global_grid_world_view.obstacles.points_type", "point_cloud2"),
                              rclcpp::Parameter("global_grid_world_view.obstacles.scan_range_max", 10.0),
                              rclcpp::Parameter("global_grid_world_view.obstacles.max_points", 1)});
        prepare(); // Install the source before injecting its first observation.
    }
    sensor_msgs::msg::PointCloud2::SharedPtr cloud(bool blocked) {
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        msg->header.frame_id = "map";
        msg->header.stamp = node->now();
        sensor_msgs::PointCloud2Modifier modifier(*msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(blocked ? 81 : 0);
        sensor_msgs::PointCloud2Iterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
        if (blocked) for (int row = 0; row < 9; ++row) for (int col = 0; col < 9; ++col) {
            *x = 4.6f + .1f * col; *y = 3.6f + .1f * row; *z = .4f;
            ++x; ++y; ++z;
        }
        return msg;
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
TEST_F(GlobalPlannerTest, LiveObstacleReplansUnchangedMapAndClearsWhenRemoved) {
    enableLiveObstacles();
    GlobalGridWorldViewTestAccess::cloud(view, cloud(false));
    prepare();
    ASSERT_TRUE(planner.plan({state, reference}, view));
    EXPECT_TRUE(view.isWalkable({5, 4, 0}));

    GlobalGridWorldViewTestAccess::cloud(view, cloud(true));
    // Same map pointer and goal: only the cloud changed. max_points=1 must
    // not truncate the overlay, which uses the full collision observation.
    prepare();
    EXPECT_FALSE(view.isWalkable({5, 4, 0}));
    EXPECT_FALSE(view.isWalkable({5.7, 4, 0}));
    const auto detour = planner.plan({state, reference}, view);
    ASSERT_TRUE(detour);
    double deviation = 0;
    for (const auto& p : detour->poses)
        deviation = std::max(deviation, std::abs(p.position.y - 4));
    EXPECT_GT(deviation, .9);
    EXPECT_EQ(detour->poses.back(), reference.poses.back());
    EXPECT_EQ(map->data[40*100+50], 0); // The static source stays immutable.

    GlobalGridWorldViewTestAccess::cloud(view, cloud(false));
    prepare();
    EXPECT_FALSE(view.isWalkable({5, 4, 0})); // One missed return cannot erase a box.
    GlobalGridWorldViewTestAccess::expireCells(view);
    prepare();
    EXPECT_TRUE(view.isWalkable({5, 4, 0}));
    const auto cleared = planner.plan({state, reference}, view);
    ASSERT_TRUE(cleared);
    for (const auto& p : cleared->poses) EXPECT_NEAR(p.position.y, 4, .11);
}
TEST_F(GlobalPlannerTest, LiveObservationGapsHoldPositionAndFreshDataRestoresRoute) {
    enableLiveObstacles();
    const auto expect_hold = [&] {
        EXPECT_TRUE(view.awaitingObservation());
        const auto result = planner.plan({state, reference}, view);
        ASSERT_TRUE(result);
        ASSERT_EQ(result->poses.size(), 1u);
        EXPECT_EQ(result->poses.front(), state.pose);
    };
    prepare();
    expect_hold();
    auto msg = cloud(true);
    msg->header.stamp = node->now() - rclcpp::Duration::from_seconds(2);
    GlobalGridWorldViewTestAccess::cloud(view, msg);
    prepare();
    expect_hold();
    msg = cloud(true);
    msg->header.frame_id = "missing_sensor_tf";
    GlobalGridWorldViewTestAccess::cloud(view, msg);
    prepare();
    expect_hold();
    GlobalGridWorldViewTestAccess::cloud(view, cloud(false));
    prepare();
    EXPECT_FALSE(view.awaitingObservation());
    const auto route = planner.plan({state, reference}, view);
    ASSERT_TRUE(route);
    EXPECT_EQ(route->poses.back(), reference.poses.back());
    EXPECT_GT(route->poses.size(), 1u);
}
TEST_F(GlobalPlannerTest, ContinuousObservationFailureFaultsAndResetRestartsWait) {
    finenav_global_planner::Params cfg;
    cfg.observation_failure_timeout = .001;
    planner.configure(cfg);
    enableLiveObstacles();
    prepare();
    ASSERT_TRUE(planner.plan({state, reference}, view));
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    EXPECT_FALSE(planner.plan({state, reference}, view));
    planner.reset();
    EXPECT_TRUE(planner.plan({state, reference}, view));
    GlobalGridWorldViewTestAccess::cloud(view, cloud(false));
    prepare();
    ASSERT_TRUE(planner.plan({state, reference}, view));
    GlobalGridWorldViewTestAccess::cloud(view, nullptr);
    prepare();
    EXPECT_TRUE(planner.plan({state, reference}, view));
}
TEST_F(GlobalPlannerTest, LiveObstacleAtStartOrGoalDoesNotPermitUnsafeRoute) {
    enableLiveObstacles();
    GlobalGridWorldViewTestAccess::cloud(view, cloud(true));
    reference.poses.back() = pose(5, 4);
    prepare();
    EXPECT_FALSE(planner.plan({state, reference}, view));
    state.pose = pose(5, 4);
    reference.poses.back() = pose(8, 4);
    prepare();
    EXPECT_FALSE(planner.plan({state, reference}, view));
}
TEST_F(GlobalPlannerTest, EscapesRasterPaddingWithoutReducingPhysicalClearance) {
    enableLiveObstacles();
    auto msg = cloud(true);
    GlobalGridWorldViewTestAccess::cloud(view, msg);
    // Obstacle starts at x=4.6: physically safe at x=3.94, but the occupied
    // cell's conservative padding would otherwise invalidate this start.
    state.pose = pose(3.94, 4);
    prepare();
    ASSERT_TRUE(view.isWalkable({3.94, 4, 0}));
    const auto route = planner.plan({state, reference}, view);
    ASSERT_TRUE(route);
    for (const auto& p : route->poses) {
        const double dx = std::max({4.6-p.position.x, p.position.x-5.4, 0.0});
        const double dy = std::max({3.6-p.position.y, p.position.y-4.4, 0.0});
        EXPECT_GE(std::hypot(dx, dy), .51);
    }
    state.pose = pose(4.2, 4); // Inside the required physical clearance.
    prepare();
    EXPECT_FALSE(planner.plan({state, reference}, view));
}
TEST_F(GlobalPlannerTest, CanExitStaticRasterPaddingButKeepsStaticObstacleClearance) {
    enableLiveObstacles();
    GlobalGridWorldViewTestAccess::cloud(view, cloud(false));
    for (int y=0; y<2; ++y) for (int x=0; x<100; ++x) map->data[y*100+x]=100;
    state.pose = pose(2, .76);
    prepare();
    EXPECT_TRUE(planner.plan({state, reference}, view));
    state.pose = pose(2, .6);
    prepare();
    EXPECT_FALSE(planner.plan({state, reference}, view));
}
}  // namespace
}  // namespace finenav::example
