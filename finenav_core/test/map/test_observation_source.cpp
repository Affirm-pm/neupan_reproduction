// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/map/observation_source.hpp"
#include "test_helpers.hpp"

using namespace finenav::core;

class ObservationSourceTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }
    void TearDown() override {
        rclcpp::shutdown();
    }
};

TEST_F(ObservationSourceTest, DefaultPolicyValues) {
    ObservationBufferPolicy p;
    EXPECT_EQ(p.max_buffer_size, 32u);
    EXPECT_EQ(p.observation_keep_time_sec, 0.0);
    EXPECT_EQ(p.source_timeout_sec, 0.0);
    EXPECT_EQ(p.expected_update_rate_hz, 0.0);
}

TEST_F(ObservationSourceTest, EnqueueAndApplyBuffered) {
    auto logger = rclcpp::get_logger("test");
    MockStateProvider provider;
    provider.state = RobotState{};
    SmokeMap map;

    int call_count = 0;
    ObservationSource<SmokeMap, SmokeCell> src(
        "test", {}, [&](const SmokeCell&, const rclcpp::Time& stamp,
                         const RobotState&, SmokeMap&) { ++call_count; },
        logger);

    rclcpp::Time t(1, 0, RCL_ROS_TIME);
    src.enqueueThreadSafe(SmokeCell{}, t);
    src.enqueueThreadSafe(SmokeCell{}, t);

    src.applyBuffered(map, t, &provider);
    EXPECT_EQ(call_count, 2);
}

TEST_F(ObservationSourceTest, BufferOverflowDropsOldest) {
    auto logger = rclcpp::get_logger("test");
    MockStateProvider provider;
    provider.state = RobotState{};
    SmokeMap map;

    ObservationBufferPolicy policy;
    policy.max_buffer_size = 2;

    double last_val = -1;
    ObservationSource<SmokeMap, SmokeCell> src(
        "test", policy,
        [&](const SmokeCell& c, const rclcpp::Time&,
            const RobotState&, SmokeMap&) { last_val = c.value; },
        logger);

    SmokeCell a{1.0}, b{2.0}, c{3.0};
    rclcpp::Time t(1, 0, RCL_ROS_TIME);
    src.enqueueThreadSafe(std::move(a), t);
    src.enqueueThreadSafe(std::move(b), t);
    src.enqueueThreadSafe(std::move(c), t); // a is dropped

    src.applyBuffered(map, t, &provider);
    EXPECT_DOUBLE_EQ(last_val, 3.0); // c(3.0) is last
}

TEST_F(ObservationSourceTest, SourceTimeoutClearsBuffer) {
    auto logger = rclcpp::get_logger("test");
    MockStateProvider provider;
    provider.state = RobotState{};
    SmokeMap map;

    ObservationBufferPolicy policy;
    policy.source_timeout_sec = 0.1;

    int call_count = 0;
    ObservationSource<SmokeMap, SmokeCell> src(
        "test", policy,
        [&](const SmokeCell&, const rclcpp::Time&,
            const RobotState&, SmokeMap&) { ++call_count; },
        logger);

    rclcpp::Time t0(1, 0, RCL_ROS_TIME);
    src.enqueueThreadSafe(SmokeCell{}, t0);

    // 0.5s later → timed out
    rclcpp::Time t1(1, 500000000, RCL_ROS_TIME);
    src.applyBuffered(map, t1, &provider);
    EXPECT_EQ(call_count, 0);
}

TEST_F(ObservationSourceTest, ExpiryPrunesOldMessages) {
    auto logger = rclcpp::get_logger("test");
    MockStateProvider provider;
    provider.state = RobotState{};
    SmokeMap map;

    ObservationBufferPolicy policy;
    policy.observation_keep_time_sec = 0.2;

    int call_count = 0;
    ObservationSource<SmokeMap, SmokeCell> src(
        "test", policy,
        [&](const SmokeCell&, const rclcpp::Time&,
            const RobotState&, SmokeMap&) { ++call_count; },
        logger);

    rclcpp::Time t0(1, 0, RCL_ROS_TIME);
    src.enqueueThreadSafe(SmokeCell{}, t0);

    // 0.5s later → expired
    rclcpp::Time t1(1, 500000000, RCL_ROS_TIME);
    src.applyBuffered(map, t1, &provider);
    EXPECT_EQ(call_count, 0);
}

TEST_F(ObservationSourceTest, NoStateProviderSkipsMessages) {
    auto logger = rclcpp::get_logger("test");
    SmokeMap map;

    int call_count = 0;
    ObservationSource<SmokeMap, SmokeCell> src(
        "test", {},
        [&](const SmokeCell&, const rclcpp::Time&,
            const RobotState&, SmokeMap&) { ++call_count; },
        logger);

    src.enqueueThreadSafe(SmokeCell{}, rclcpp::Time(1, 0, RCL_ROS_TIME));
    src.applyBuffered(map, rclcpp::Time(1, 0, RCL_ROS_TIME), nullptr);
    EXPECT_EQ(call_count, 0);
}

TEST_F(ObservationSourceTest, NoFuserWarns) {
    auto logger = rclcpp::get_logger("test");
    SmokeMap map;

    ObservationSource<SmokeMap, SmokeCell> src(
        "test", {}, nullptr, logger);
    src.enqueueThreadSafe(SmokeCell{}, rclcpp::Time(1, 0, RCL_ROS_TIME));
    // Should not crash — just WARN_ONCE.
    src.applyBuffered(map, rclcpp::Time(1, 0, RCL_ROS_TIME), nullptr);
}

TEST_F(ObservationSourceTest, StarvationWarns) {
    auto logger = rclcpp::get_logger("test");
    MockStateProvider provider;
    provider.state = RobotState{};
    SmokeMap map;

    ObservationBufferPolicy policy;
    policy.expected_update_rate_hz = 100.0; // expect data every 10ms

    ObservationSource<SmokeMap, SmokeCell> src(
        "test", policy,
        [](const SmokeCell&, const rclcpp::Time&,
           const RobotState&, SmokeMap&) {},
        logger);

    // Enqueue once, then wait 5s → starvation
    src.enqueueThreadSafe(SmokeCell{}, rclcpp::Time(1, 0, RCL_ROS_TIME));
    src.applyBuffered(map, rclcpp::Time(6, 0, RCL_ROS_TIME), &provider);
    // Not crashing is the main assertion — starvation WARN is logged.
}
