#include <gtest/gtest.h>
#include "planner_safety.hpp"
#include <vector>
using namespace finenav::example;
struct Point { double x,y; };
TEST(FailureWatch, StopsFirstAndFaultsOnlyContinuousFailure) {
    FailureWatch w; const auto t=FailureWatch::Clock::now();
    EXPECT_FALSE(w.expired(t,1));
    EXPECT_FALSE(w.expired(t+std::chrono::milliseconds(900),1));
    EXPECT_TRUE(w.expired(t+std::chrono::seconds(1),1));
    w.reset(); EXPECT_FALSE(w.expired(t+std::chrono::seconds(2),1));
}
TEST(SweptFootprint, SeesNearPointAmongThousands) {
    std::vector<Point> p(1001,{0,5}); p[5]={.36,0};
    EXPECT_LT(sweptClearance(p,0,0,0,.6,0,.7,.6,0,8,3,.1),.02);
}
TEST(SweptFootprint, ChecksBrakingAheadAndReverse) {
    const std::vector<Point> front{{.5,0}},back{{-.5,0}},clear{{0,5}};
    EXPECT_LT(sweptClearance(front,0,0,0,1,0,.7,.6,0,1,3,.1),.02);
    EXPECT_LT(sweptClearance(back,0,0,0,-1,0,.7,.6,0,1,3,.1),.02);
    EXPECT_GT(sweptClearance(clear,0,0,0,.6,.3,.7,.6,0,8,3,.1),.02);
}
TEST(SweptFootprint, RejectsNonfiniteStateAndCoversCornerSweep) {
    const std::vector<Point> p{{.32,.32}};
    EXPECT_LT(sweptClearance(p,0,0,0,0,.6,.7,.6,0,8,3,.1),.02);
    EXPECT_EQ(sweptClearance(p,0,0,0,NAN,0,.7,.6,0,8,3,.1),0);
}
