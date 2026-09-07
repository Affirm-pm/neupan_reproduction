// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

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

  const auto & noisesVx() const {return noises_vx_;}
  const auto & noisesVy() const {return noises_vy_;}
  const auto & noisesWz() const {return noises_wz_;}

  models::State populateNoisedState()
  {
    models::State state;
    state.reset(settings_.batch_size, settings_.time_steps);
    models::ControlSequence controls;
    controls.reset(settings_.time_steps);
    setNoisedControls(state, controls);
    return state;
  }

  bool waitForNoises(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      {
        std::lock_guard<std::mutex> guard(noise_lock_);
        const auto expected_size = settings_.batch_size * settings_.time_steps;
        if (noises_vx_.size() == expected_size && noises_vy_.size() == expected_size &&
          noises_wz_.size() == expected_size)
        {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
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

template<typename Tensor>
void expectTwoDimensionalShape(const Tensor & tensor, std::size_t batch_size, std::size_t time_steps)
{
  ASSERT_EQ(tensor.rows(), batch_size);
  ASSERT_EQ(tensor.cols(), time_steps);
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

TEST(NoiseGeneratorTest, RegenerationModePopulatesNoisesAfterNotification)
{
  auto settings = makeSettings();
  TestNoiseGenerator generator;
  generator.initialize(settings, true, true);

  generator.generateNextNoises();

  ASSERT_TRUE(generator.waitForNoises(std::chrono::milliseconds(100)));
  expectTwoDimensionalShape(generator.noisesVx(), 2, 3);
  expectTwoDimensionalShape(generator.noisesVy(), 2, 3);
  expectTwoDimensionalShape(generator.noisesWz(), 2, 3);
  generator.shutdown();
}

TEST(NoiseGeneratorTest, RegenerationModeResetSchedulesNoisesWithNewShape)
{
  auto settings = makeSettings();
  TestNoiseGenerator generator;
  generator.initialize(settings, true, true);
  settings.batch_size = 3;
  settings.time_steps = 4;

  generator.reset(settings, true);

  ASSERT_TRUE(generator.waitForNoises(std::chrono::milliseconds(100)));
  expectTwoDimensionalShape(generator.noisesVx(), 3, 4);
  expectTwoDimensionalShape(generator.noisesVy(), 3, 4);
  expectTwoDimensionalShape(generator.noisesWz(), 3, 4);
  generator.shutdown();
}

TEST(NoiseGeneratorTest, SynchronousModePopulatesFiniteNoiseTensors)
{
  auto settings = makeSettings();
  TestNoiseGenerator generator;

  generator.initialize(settings, true, false);

  expectTwoDimensionalShape(generator.noisesVx(), 2, 3);
  expectTwoDimensionalShape(generator.noisesVy(), 2, 3);
  expectTwoDimensionalShape(generator.noisesWz(), 2, 3);
  EXPECT_TRUE(generator.noisesVx().allFinite());
  EXPECT_TRUE(generator.noisesVy().allFinite());
  EXPECT_TRUE(generator.noisesWz().allFinite());

  generator.shutdown();
}

TEST(NoiseGeneratorTest, ResetResizesNoiseTensorsAndPopulatesState)
{
  auto settings = makeSettings();
  TestNoiseGenerator generator;
  generator.initialize(settings, true, false);

  settings.batch_size = 3;
  settings.time_steps = 4;
  generator.reset(settings, true);
  const auto state = generator.populateNoisedState();

  expectTwoDimensionalShape(generator.noisesVx(), 3, 4);
  expectTwoDimensionalShape(generator.noisesVy(), 3, 4);
  expectTwoDimensionalShape(generator.noisesWz(), 3, 4);
  expectTwoDimensionalShape(state.cvx, 3, 4);
  expectTwoDimensionalShape(state.cvy, 3, 4);
  expectTwoDimensionalShape(state.cwz, 3, 4);
  EXPECT_TRUE(state.cvx.allFinite());
  EXPECT_TRUE(state.cvy.allFinite());
  EXPECT_TRUE(state.cwz.allFinite());

  generator.shutdown();
}

}  // namespace
}  // namespace mppi
