#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "neupan_planner.hpp"
#include "neupan/neupan_planner.hpp"
using namespace finenav::example;
struct World : NeuPanPlanner::IWorldView {
    bool valid=true;
    std::vector<finenav::common::Position3D> points,full;
    bool observationValid() const override {return valid;}
    const std::vector<finenav::common::Position3D>& obstaclePoints() const override {return points;}
    const std::vector<finenav::common::Position3D>& collisionPoints() const override {return full;}
};
struct PluginTest : testing::Test {
    NeuPanPlanner planner;
    neupan_planner::Params cfg;
    finenav::core::RobotState state{};
    finenav::core::InputTrajectoryT<NeuPanPlanner> ref;
    World world;
    void SetUp() override {
        cfg.dune_checkpoint=TEST_MODEL;
        cfg.ref_speed=.6;cfg.d_max=.3;cfg.d_min=.05;cfg.ro_obs=10;cfg.bk=0;
        cfg.collision_threshold=.02;cfg.failure_timeout=.01;
        planner.configure(cfg);state.pose.orientation.w=1;
        geometry_msgs::msg::Pose goal;goal.position.x=3;goal.orientation.w=1;
        ref.poses.push_back(goal);
    }
    NeuPanPlanner::Result step(){return planner.plan({state,ref},world);}
    void expectZero(const NeuPanPlanner::Result& out) {
        ASSERT_TRUE(out);ASSERT_EQ(out->twists.size(),1u);
        EXPECT_EQ(out->twists[0].linear.x,0);EXPECT_EQ(out->twists[0].angular.z,0);
    }
};
TEST_F(PluginTest, ValidEmptyObservationCanDriveButMissingObservationStops) {
    auto out=step();ASSERT_TRUE(out);EXPECT_GT(out->twists[0].linear.x,.1);
    world.valid=false;expectZero(step());
    std::this_thread::sleep_for(std::chrono::milliseconds(15));EXPECT_FALSE(step());
    planner.reset();world.valid=true;out=step();ASSERT_TRUE(out);EXPECT_GT(out->twists[0].linear.x,.1);
}
TEST_F(PluginTest, FullCloudStopsEvenWhenOptimizerHasNoNearPoint) {
    world.points.assign(1000,{0,5,0});world.full=world.points;world.full[5]={.36,0,0};
    expectZero(step());
    std::this_thread::sleep_for(std::chrono::milliseconds(15));EXPECT_FALSE(step());
}
TEST_F(PluginTest, BlockedTerminalYawStopsThenFaults) {
    ref.poses[0].position.x=0;ref.poses[0].orientation.w=std::cos(.5);ref.poses[0].orientation.z=std::sin(.5);
    world.full={{.32,.32,0}};expectZero(step());
    std::this_thread::sleep_for(std::chrono::milliseconds(15));EXPECT_FALSE(step());
}
TEST_F(PluginTest, TerminalYawWithoutMeasuredProgressEventuallyFaults) {
    ref.poses[0].position.x=0;ref.poses[0].orientation.w=std::cos(.5);ref.poses[0].orientation.z=std::sin(.5);
    ASSERT_TRUE(step());std::this_thread::sleep_for(std::chrono::milliseconds(15));
    expectZero(step());EXPECT_FALSE(step());
}
TEST_F(PluginTest, MissingModelStopsThenFaults) {
    cfg.dune_checkpoint="/missing-neupan-model.bin";planner.configure(cfg);expectZero(step());
    std::this_thread::sleep_for(std::chrono::milliseconds(15));EXPECT_FALSE(step());
}
TEST_F(PluginTest, DefaultModelResolvesBesideLoadedPlugin) {
    cfg.dune_checkpoint="";planner.configure(cfg);
    auto out=step();ASSERT_TRUE(out);EXPECT_GT(out->twists[0].linear.x,.1);
}

TEST_F(PluginTest, FailedSolveNeverExecutesPreviousNonzeroNominal) {
    // Poor conditioning deterministically exercises OSQP's iteration failure
    // with an otherwise valid configuration. Confirm the failure in the real
    // core, rather than guessing from a zero command.
    cfg.ro_obs=1e6;planner.configure(cfg);
    neupan::NeuPANPlanner::Config core;
    core.length=.7;core.width=.6;core.ref_speed=.6;core.collision_threshold=.02;
    core.adjust.d_max=.3;core.adjust.d_min=.05;core.adjust.bk=0;core.adjust.ro_obs=cfg.ro_obs;
    core.dune_checkpoint=TEST_MODEL;
    neupan::NeuPANPlanner probe(core);
    probe.updateInitialPathFromGoal({0,0,0},{3,0,0});
    neupan::NeuPANPlanner::Info info;neupan::Mat2X pts(2,0);
    probe.forward({0,0,0},pts,info);expectZero(step());
    pts.resize(2,20);
    for(int i=0;i<20;++i){pts.col(i)=neupan::Vec2(.55,-.3+.03*i);world.points.push_back({.55,-.3+.03*i,0});}
    world.full=world.points;
    probe.forward({0,0,0},pts,info);ASSERT_TRUE(info.solved);
    const auto moving=step();ASSERT_TRUE(moving);ASSERT_GT(std::abs(moving->twists[0].linear.x),.1);
    pts.resize(2,0);const auto nominal=probe.forward({0,0,0},pts,info);
    ASSERT_FALSE(info.solved);ASSERT_GT(std::abs(nominal.x()),.1);
    world.points.clear();world.full.clear();expectZero(step());
}

TEST_F(PluginTest, RecoveryBudgetExhaustionStopsThenFaultsAndNewGoalRestarts) {
    cfg.stall_timeout=.001;cfg.failure_timeout=1;planner.configure(cfg);
    ASSERT_TRUE(step());
    for(int i=0;i<5;++i){std::this_thread::sleep_for(std::chrono::milliseconds(3));ASSERT_TRUE(step());}
    std::this_thread::sleep_for(std::chrono::milliseconds(3));expectZero(step());
    EXPECT_FALSE(step());
    ref.poses[0].position.x=3.1;
    const auto out=step();ASSERT_TRUE(out);EXPECT_GT(out->twists[0].linear.x,.1);
}

TEST_F(PluginTest, StationaryMultiPosePathStopsThenFaults) {
    geometry_msgs::msg::Pose start;start.orientation.w=1;
    ref.poses.insert(ref.poses.begin(),start);
    const auto moving=step();ASSERT_TRUE(moving);EXPECT_GT(moving->twists[0].linear.x,.1);
    std::this_thread::sleep_for(std::chrono::milliseconds(15));expectZero(step());
    EXPECT_FALSE(step());
}
