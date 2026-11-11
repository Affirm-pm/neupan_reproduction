#include <gtest/gtest.h>
#include "obstacle_bypass.hpp"
#include <vector>
using finenav::example::ObstacleBypass;
using Point=ObstacleBypass::Point;
using Change=ObstacleBypass::Change;
std::vector<Point> cylinder(double x=2,double y=0) {
    std::vector<Point> out;
    for(int i=0;i<80;++i){const double a=2*std::acos(-1.)*i/80;out.push_back({x+.3*std::cos(a),y+.3*std::sin(a)});}
    return out;
}
TEST(ObstacleBypass, SelectsSafeReferenceImmediatelyWithoutStallTimer) {
    ObstacleBypass b;auto p=cylinder();
    ASSERT_EQ(b.update({0,0},{5,0},0,p,.7,.6,0),Change::Installed);
    EXPECT_TRUE(b.active);EXPECT_NE(b.side,0);
    for(int i=0;i<3;++i) EXPECT_TRUE(ObstacleBypass::clearSegment(b.route[i],b.route[i+1],p,std::hypot(.35,.3)+.1));
}
TEST(ObstacleBypass, LatchesSideAndDoesNotReplaceRouteEachFrame) {
    ObstacleBypass b;auto p=cylinder();ASSERT_EQ(b.update({0,0},{5,0},0,p,.7,.6,0),Change::Installed);
    const auto route=b.route;const int side=b.side;
    for(int i=0;i<10;++i){p.push_back({1.8,i*.001});EXPECT_EQ(b.update({.01*i,.01},{5,0},0,p,.7,.6,0),Change::None);EXPECT_EQ(b.side,side);EXPECT_EQ(b.route[1].y,route[1].y);}
}
TEST(ObstacleBypass, ChoosesOtherSideWhenOneSideIsBlocked) {
    ObstacleBypass b;auto p=cylinder();
    for(int i=0;i<100;++i)p.push_back({.05*i,1.25});
    ASSERT_EQ(b.update({0,0},{5,0},0,p,.7,.6,0),Change::Installed);EXPECT_EQ(b.side,-1);
}
TEST(ObstacleBypass, RejoinsOnlyAfterPassingAndClearRejoin) {
    ObstacleBypass b;auto p=cylinder();ASSERT_EQ(b.update({0,0},{5,0},0,p,.7,.6,0),Change::Installed);
    EXPECT_EQ(b.update({.2,0},{5,0},0,p,.7,.6,0),Change::None);
    EXPECT_EQ(b.update(b.route[2],{5,0},0,p,.7,.6,0),Change::Cleared);EXPECT_FALSE(b.active);
}
TEST(ObstacleBypass, DoesNotInventRouteThroughWallOrForClearGoal) {
    ObstacleBypass b;std::vector<Point> p;
    EXPECT_EQ(b.update({0,0},{5,0},0,p,.7,.6,0),Change::None);
    for(int i=0;i<200;++i)p.push_back({2,-5+.05*i});
    EXPECT_EQ(b.update({0,0},{5,0},0,p,.7,.6,0),Change::None);EXPECT_FALSE(b.active);
    b.options.enabled=false;EXPECT_EQ(b.update({0,0},{5,0},0,cylinder(),.7,.6,0),Change::None);
}
