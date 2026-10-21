// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include "neupan/tensor_io.hpp"
#include "neupan_plugin.hpp"
using namespace finenav::example;
class View : public NeuPANView {
public:
    bool valid=true;
    neupan::Mat2X cloud{2,0};
    bool ready() const override { return valid; }
    const neupan::Mat2X& points() const override { return cloud; }
};
class AdapterTest : public ::testing::Test {
protected:
    NeuPANPlanner planner;
    finenav::core::RobotState state;
    finenav::core::InputTrajectoryT<NeuPANPlanner> ref;
    View view;
    void SetUp() override {
        neupan_planner::Params p;
        p.config_file=std::string(NEUPAN_ASSET_DIR)+"/planner.yaml";
        p.model_file=std::string(NEUPAN_ASSET_DIR)+"/models/diff_0p7_0p6.bin";
        planner.configure(p);
        state.pose.orientation.w=1;
        geometry_msgs::msg::Pose goal;
        goal.position.x=2;
        goal.orientation.w=1;
        ref.poses.push_back(goal);
    }
    geometry_msgs::msg::Twist command() { return planner.plan({state,ref},view)->twists.front(); }
};
TEST_F(AdapterTest, MissingSensorStops) {
    view.valid=false;
    auto c=command(); EXPECT_EQ(c.linear.x,0); EXPECT_EQ(c.angular.z,0);
}
TEST_F(AdapterTest, OpenSpaceMovesWithinLimits) {
    auto c=command(); EXPECT_GT(c.linear.x,0); EXPECT_LE(c.linear.x,0.3);
    EXPECT_LE(std::abs(c.angular.z),0.6);
}
TEST_F(AdapterTest, FootprintCollisionStops) {
    view.cloud.resize(2,1); view.cloud.col(0)<<0.1,0;
    auto c=command(); EXPECT_EQ(c.linear.x,0); EXPECT_EQ(c.angular.z,0);
}
TEST_F(AdapterTest, GoalYawAndWrap) {
    ref.poses[0].position.x=0;
    ref.poses[0].orientation.z=std::sin(0.5);
    ref.poses[0].orientation.w=std::cos(0.5);
    auto c=command(); EXPECT_EQ(c.linear.x,0); EXPECT_GT(c.angular.z,0);
    state.pose=ref.poses[0]; c=command(); EXPECT_EQ(c.linear.x,0); EXPECT_EQ(c.angular.z,0);
}
TEST_F(AdapterTest, InvalidInputStops) {
    state.pose.position.x=std::numeric_limits<double>::quiet_NaN();
    auto c=command(); EXPECT_EQ(c.linear.x,0); EXPECT_EQ(c.angular.z,0);
}
TEST_F(AdapterTest, SweptRotationUsesRectangle) {
    neupan::Mat2X points(2,1); points.col(0)<<0.48,0;
    EXPECT_TRUE(planner.collisionFree({0,0,0},{0,0},points));
    EXPECT_FALSE(planner.collisionFree({0,0,0},{0,0.6},points));
}
TEST(ModelGeometry, RejectsWrongFootprint) {
    auto p=neupan::NeuPANPlanner::fromYaml(std::string(NEUPAN_ASSET_DIR)+"/planner.yaml",
                                         std::string(NEUPAN_ASSET_DIR)+"/models/diff_0p7_0p6.bin");
    auto cfg=p.config(); cfg.length=0.5;
    EXPECT_THROW(neupan::NeuPANPlanner other(cfg),std::exception);
}
TEST_F(AdapterTest, ClosedLoopAvoidsOffsetObstacle) {
    view.cloud.resize(2,16);
    for (int i=0;i<16;++i) {
        const double angle=2*M_PI*i/16;
        view.cloud.col(i)<<0.9+0.07*std::cos(angle),0.43+0.07*std::sin(angle);
    }
    double heading=0, max_detour=0;
    for (int step=0;step<500;++step) {
        auto c=command();
        state.pose.position.x+=0.1*c.linear.x*std::cos(heading);
        state.pose.position.y+=0.1*c.linear.x*std::sin(heading);
        heading+=0.1*c.angular.z;
        state.pose.orientation.z=std::sin(heading/2);
        state.pose.orientation.w=std::cos(heading/2);
        state.twist=c;
        max_detour=std::max(max_detour,std::abs(state.pose.position.y));
        ASSERT_TRUE(planner.collisionFree({state.pose.position.x,state.pose.position.y,heading},{0,0},view.cloud));
        if (std::hypot(state.pose.position.x-2,state.pose.position.y)<0.08) break;
    }
    EXPECT_LT(std::hypot(state.pose.position.x-2,state.pose.position.y),0.08);
    EXPECT_GT(max_detour,0.05);
}
TEST_F(AdapterTest, DirectBlockageMayStopAtLocalMinimumButNeverCollides) {
    view.cloud.resize(2,16);
    for (int i=0;i<16;++i) {
        const double angle=2*M_PI*i/16;
        view.cloud.col(i)<<0.9+0.07*std::cos(angle),0.18+0.07*std::sin(angle);
    }
    double heading=0;
    geometry_msgs::msg::Twist c;
    for (int step=0;step<200;++step) {
        c=command();
        state.pose.position.x+=0.1*c.linear.x*std::cos(heading);
        state.pose.position.y+=0.1*c.linear.x*std::sin(heading);
        heading+=0.1*c.angular.z;
        state.pose.orientation.z=std::sin(heading/2);state.pose.orientation.w=std::cos(heading/2);
        state.twist=c;
        ASSERT_TRUE(planner.collisionFree({state.pose.position.x,state.pose.position.y,heading},{0,0},view.cloud));
    }
    EXPECT_LT(std::abs(c.linear.x),0.01);
    EXPECT_LT(state.pose.position.x,0.6);
}
TEST_F(AdapterTest, FirstCommandHonorsMeasuredAcceleration) {
    auto c=command(); EXPECT_LE(std::abs(c.linear.x),0.050001);
    EXPECT_LE(std::abs(c.angular.z),0.100001);
}
TEST(ModelExport, CppMatchesPyTorchForTrainedRectangle) {
    const auto model=neupan::MLP::load(std::string(NEUPAN_ASSET_DIR)+"/models/diff_0p7_0p6.bin");
    const auto reference=neupan::TensorFile::load(std::string(NEUPAN_ASSET_DIR)+"/models/validation.nptf");
    auto result=model.forward(reference.at("points").cast<float>());
    EXPECT_LT((result.cast<double>()-reference.at("mu")).cwiseAbs().maxCoeff(),1e-4);
}
TEST(ModelExport, OriginalSimulatorGeometryAndPyTorchMatch) {
    const auto base=std::string(NEUPAN_ASSET_DIR)+"/";
    const auto planner=neupan::NeuPANPlanner::fromYaml(base+"planner.yaml", base+"models/diff_0p7_0p6.bin");
    EXPECT_DOUBLE_EQ(planner.config().length,0.7);
    EXPECT_DOUBLE_EQ(planner.config().width,0.6);
    const auto model=neupan::MLP::load(base+"models/diff_0p7_0p6.bin");
    const auto reference=neupan::TensorFile::load(base+"models/validation.nptf");
    auto result=model.forward(reference.at("points").cast<float>());
    EXPECT_LT((result.cast<double>()-reference.at("mu")).cwiseAbs().maxCoeff(),1e-4);
}
