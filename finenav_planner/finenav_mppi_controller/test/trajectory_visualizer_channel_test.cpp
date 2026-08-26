// Copyright (c) 2026 IWIN-FINS Lab, Shanghai Jiao Tong University.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include "finenav_mppi_controller/tools/trajectory_visualizer.hpp"

namespace
{

mppi::VisualizationFrame::SharedPtr makeFrame(const std::uint64_t generation)
{
  auto frame = std::make_shared<mppi::VisualizationFrame>();
  frame->generation = generation;
  return frame;
}

TEST(TrajectoryVisualizerChannelTest, NewFrameReplacesOlderFrame)
{
  mppi::LatestFrameChannel channel;
  channel.submit(makeFrame(1));
  channel.submit(makeFrame(2));

  const auto event = channel.wait();
  ASSERT_TRUE(event.has_value());
  ASSERT_EQ(event->type, mppi::VisualizationEvent::Type::Frame);
  ASSERT_TRUE(event->frame);
  EXPECT_EQ(event->frame->generation, 2U);
}

TEST(TrajectoryVisualizerChannelTest, ClearDropsPendingFrames)
{
  mppi::LatestFrameChannel channel;
  channel.submit(makeFrame(1));
  channel.clear(2);

  const auto event = channel.wait();
  ASSERT_TRUE(event.has_value());
  EXPECT_EQ(event->type, mppi::VisualizationEvent::Type::Clear);
  EXPECT_EQ(event->generation, 2U);

  channel.stop();
  const auto stopped = channel.wait();
  ASSERT_TRUE(stopped.has_value());
  EXPECT_EQ(stopped->type, mppi::VisualizationEvent::Type::Stop);
}

TEST(TrajectoryVisualizerChannelTest, StopWakesWaitingConsumer)
{
  mppi::LatestFrameChannel channel;
  std::promise<mppi::VisualizationEvent::Type> result_promise;
  auto result_future = result_promise.get_future();

  std::thread consumer([&channel, &result_promise]() {
    const auto event = channel.wait();
    ASSERT_TRUE(event.has_value());
    result_promise.set_value(event->type);
  });

  EXPECT_EQ(result_future.wait_for(std::chrono::milliseconds(20)),
    std::future_status::timeout);
  channel.stop();
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(result_future.get(), mppi::VisualizationEvent::Type::Stop);
  consumer.join();
}

TEST(TrajectoryVisualizerChannelTest, SubmitAfterStopIsIgnored)
{
  mppi::LatestFrameChannel channel;
  channel.stop();
  channel.submit(makeFrame(3));

  const auto event = channel.wait();
  ASSERT_TRUE(event.has_value());
  EXPECT_EQ(event->type, mppi::VisualizationEvent::Type::Stop);
}

TEST(TrajectoryVisualizerChannelTest, RestartReopensStoppedChannel)
{
  mppi::LatestFrameChannel channel;
  channel.stop();
  channel.restart();
  channel.submit(makeFrame(4));

  const auto event = channel.wait();
  ASSERT_TRUE(event.has_value());
  ASSERT_EQ(event->type, mppi::VisualizationEvent::Type::Frame);
  ASSERT_TRUE(event->frame);
  EXPECT_EQ(event->frame->generation, 4U);
}

}  // namespace
