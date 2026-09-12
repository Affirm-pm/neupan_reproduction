// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "finenav_core/common/detail/algo_register_details.hpp"

namespace {

/// An algorithm that OPTS IN to the framework's services.
struct OptedIn {
    rclcpp::Logger logger{rclcpp::get_logger("self_named")};
    finenav::viz::FineNavVisualizer viz;
    int visualizer_calls = 0;

    void setLogger(const rclcpp::Logger& l) { logger = l; }
    void setVisualizer(const finenav::viz::FineNavVisualizer& v) { viz = v; ++visualizer_calls; }
};

/// An algorithm that OPTS OUT.  The injectors are still called on it, so they
/// must compile to nothing rather than to an error — that is the entire reason
/// the probes exist.
struct OptedOut {};

class AlgoRegisterDetailsTest : public ::testing::Test {
protected:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};

}  // namespace

// Regression guard.  The probe used to read `requires { algo.setLogger(
// rclcpp::Logger{}); }`, which is ill-formed — both of Logger's constructors
// are private — so it was always false and injection silently never ran: every
// algorithm kept the logger it made for itself.  Asking about the TYPE with
// declval is the only form that works, and this test is what says so.
TEST_F(AlgoRegisterDetailsTest, InjectsNodeScopedLogger)
{
    auto node = std::make_shared<rclcpp::Node>("probe_node");

    OptedIn algo;
    ASSERT_STREQ(algo.logger.get_name(), "self_named");

    finenav::core::detail::injectLogger(node, algo, "dummy_map");

    ASSERT_NE(algo.logger.get_name(), nullptr);
    EXPECT_EQ(std::string{algo.logger.get_name()}, "probe_node.dummy_map");
}

TEST_F(AlgoRegisterDetailsTest, InjectsVisualizerAndReturnsIt)
{
    auto node = std::make_shared<rclcpp::Node>("probe_node");

    OptedIn algo;
    EXPECT_EQ(algo.visualizer_calls, 0);

    auto viz = finenav::core::detail::injectVisualizer(node, algo);

    EXPECT_EQ(algo.visualizer_calls, 1);
    EXPECT_NE(viz, nullptr);
}

// An algorithm without the setters must be left alone, not rejected: the
// injectors run for every registered algorithm.
TEST_F(AlgoRegisterDetailsTest, SkipsAlgorithmWithoutSetters)
{
    auto node = std::make_shared<rclcpp::Node>("probe_node");

    OptedOut algo;
    finenav::core::detail::injectLogger(node, algo, "dummy_map");
    EXPECT_EQ(finenav::core::detail::injectVisualizer(node, algo), nullptr);
}
