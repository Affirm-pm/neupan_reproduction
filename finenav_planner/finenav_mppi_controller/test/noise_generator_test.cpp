// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <mutex>

#include "finenav_mppi_controller/tools/noise_generator.hpp"

namespace mppi
{
namespace
{

class TestNoiseGenerator : public NoiseGenerator
{
public:
  bool isThreadJoinable() const
  {
    return noise_thread_.joinable();
  }

  bool regeneratesNoises() const
  {
    return regenerate_noises_;
  }

  bool isActive()
  {
    std::lock_guard<std::mutex> guard(noise_lock_);
    return active_;
  }
};

models::OptimizerSettings makeSettings()
{
  models::OptimizerSettings settings;
  settings.batch_size = 2;
  settings.time_steps = 3;
  settings.sampling_std = {0.2f, 0.2f, 0.4f};
  return settings;
}

TEST(NoiseGeneratorTest, SynchronousModeDoesNotStartWorker)
{
  auto settings = makeSettings();
  TestNoiseGenerator generator;

  generator.initialize(settings, false, false);

  EXPECT_FALSE(generator.regeneratesNoises());
  EXPECT_FALSE(generator.isThreadJoinable());
  generator.shutdown();
  EXPECT_FALSE(generator.isActive());
}

TEST(NoiseGeneratorTest, RegenerationModeStartsAndStopsWorker)
{
  auto settings = makeSettings();
  TestNoiseGenerator generator;

  generator.initialize(settings, false, true);

  EXPECT_TRUE(generator.regeneratesNoises());
  EXPECT_TRUE(generator.isThreadJoinable());
  generator.shutdown();
  EXPECT_FALSE(generator.isThreadJoinable());
  EXPECT_FALSE(generator.isActive());
}

TEST(NoiseGeneratorTest, RegenerationModeCanBeReinitialized)
{
  auto settings = makeSettings();
  TestNoiseGenerator generator;

  generator.initialize(settings, false, true);
  ASSERT_TRUE(generator.isThreadJoinable());

  generator.initialize(settings, false, true);

  EXPECT_TRUE(generator.isThreadJoinable());
  generator.shutdown();
  EXPECT_FALSE(generator.isThreadJoinable());
  EXPECT_FALSE(generator.isActive());
}

}  // namespace
}  // namespace mppi
