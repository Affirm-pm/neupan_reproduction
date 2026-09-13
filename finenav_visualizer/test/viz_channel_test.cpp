// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Pulled in by the finenav header below, so it has to be parsed BEFORE the
// access hack: `#define private public` is only meant to open finenav's own
// declarations, not a third-party library's internals.
#include <fmt/format.h>

#define private public
#include "finenav_visualizer/viz_channel.hpp"
#undef private

namespace
{

/// A channel whose only job is to count how often its message is TAKEN.
///
/// Taking is the half the subscription gate exists to skip: the staged data is
/// converted into its wire form and then serialised, and with nobody to receive
/// it both are pure waste.  A gate that skipped only the send would be
/// invisible here, which is the point — this counts the expensive step.
class ProbeChannel : public finenav::viz::TypedChannel<std_msgs::msg::String> {
public:
  // The two state queries stay public HERE, though the base keeps them
  // protected: a test has to drive them, and access is read off the static
  // type, so widening the override costs the real interface nothing.
  void stage() { staged_ = true; }
  bool empty() const override { return !staged_; }
  void clear() override { staged_ = false; }

  int takes{0};

private:
  std::string subtopic() const override { return "/probe"; }

  std_msgs::msg::String takeMessage(const rclcpp::Time&) override {
    ++takes;
    return std_msgs::msg::String{};
  }

  bool staged_{false};
};

}  // namespace

// An unwatched publish is not a cheap no-op: it is a full conversion and
// serialise of data the middleware then drops.  With no subscriber anywhere,
// none of that may happen — and the publisher is still created, so the cycle
// after this one is already gated rather than paying for the publisher again.
TEST(VizChannelTest, TakesNothingWhenNobodyIsSubscribed)
{
  auto node = std::make_shared<rclcpp::Node>("viz_channel_unwatched");
  ProbeChannel channel;

  channel.stage();
  channel.publishStaged(*node, "finenav_probe_unwatched", node->get_clock()->now());

  EXPECT_EQ(channel.takes, 0);
  EXPECT_EQ(channel.pubs_.count("finenav_probe_unwatched/probe"), 1U);
}

// The gate delays the publish, it does not move the boundary: the visualizer
// still clears every channel after every publish, so an unwatched cycle leaves
// nothing staged behind it either.
TEST(VizChannelTest, GatedPublishStillLeavesTheChannelClearable)
{
  auto node = std::make_shared<rclcpp::Node>("viz_channel_unwatched_clear");
  ProbeChannel channel;

  channel.stage();
  channel.publishStaged(*node, "finenav_probe_unwatched", node->get_clock()->now());
  channel.clear();

  EXPECT_TRUE(channel.empty());
  EXPECT_EQ(channel.takes, 0);
}

// A channel that stages nothing keeps its old contract exactly: no message, and
// no publisher either — the gate is downstream of the empty check, not instead
// of it.
TEST(VizChannelTest, EmptyChannelCreatesNoPublisher)
{
  auto node = std::make_shared<rclcpp::Node>("viz_channel_empty");
  ProbeChannel channel;

  channel.publishStaged(*node, "finenav_probe_empty", node->get_clock()->now());

  EXPECT_TRUE(channel.pubs_.empty());
  EXPECT_EQ(channel.takes, 0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
