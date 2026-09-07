// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "finenav_mppi_controller/tools/noise_generator.hpp"

#include <mutex>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>

namespace mppi
{

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic, bool regenerate_noises)
{
  shutdown();

  {
    std::lock_guard<std::mutex> guard(noise_lock_);
    settings_ = settings;
    is_holonomic_ = is_holonomic;
    regenerate_noises_ = regenerate_noises;
    deterministic_benchmark_noise_ =
      std::getenv("FINENAV_MPPI_BENCHMARK_DETERMINISTIC_NOISE") != nullptr;
    active_ = true;
    ready_ = false;

    noises_vx_.resize(settings_.batch_size, settings_.time_steps);
    noises_vy_.resize(settings_.batch_size, settings_.time_steps);
    noises_wz_.resize(settings_.batch_size, settings_.time_steps);
    noises_vx_.setZero();
    noises_vy_.setZero();
    noises_wz_.setZero();

    if (!regenerate_noises_) {
      generateNoisedControls();
    }
  }

  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  }
}

void NoiseGenerator::shutdown()
{
  {
    std::lock_guard<std::mutex> guard(noise_lock_);
    active_ = false;
    ready_ = true;
  }
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::lock_guard<std::mutex> guard(noise_lock_);
    if (!active_ || !regenerate_noises_) {
      return;
    }
    ready_ = true;
  }
  noise_cond_.notify_one();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  state.cvx = noises_vx_;
  state.cvy = noises_vy_;
  state.cwz = noises_wz_;
  state.cvx.rowwise() += control_sequence.vx.transpose();
  state.cvy.rowwise() += control_sequence.vy.transpose();
  state.cwz.rowwise() += control_sequence.wz.transpose();
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  bool notify_worker = false;
  {
    std::lock_guard<std::mutex> guard(noise_lock_);
    settings_ = settings;
    is_holonomic_ = is_holonomic;

    // Recompute the noises on reset, initialization, and fallback.
    noises_vx_.resize(settings_.batch_size, settings_.time_steps);
    noises_vy_.resize(settings_.batch_size, settings_.time_steps);
    noises_wz_.resize(settings_.batch_size, settings_.time_steps);
    noises_vx_.setZero();
    noises_vy_.setZero();
    noises_wz_.setZero();

    if (regenerate_noises_) {
      ready_ = true;
      notify_worker = true;
    } else {
      generateNoisedControls();
    }
  }

  if (notify_worker) {
    noise_cond_.notify_one();
  }
}

void NoiseGenerator::noiseThread()
{
  while (true) {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noise_cond_.wait(guard, [this]() {return ready_ || !active_;});
    if (!active_) {
      return;
    }
    ready_ = false;
    generateNoisedControls();
  }
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;

  if (deterministic_benchmark_noise_) {
    auto generate = [](models::BatchArray & target, float standard_deviation) {
        for (Eigen::Index row = 0; row < target.rows(); ++row) {
          for (Eigen::Index col = 0; col < target.cols(); ++col) {
            const auto index = static_cast<unsigned int>(row * 37 + col * 17);
            const float centered = static_cast<float>(index % 101u) / 50.0F - 1.0F;
            target(row, col) = standard_deviation * centered;
          }
        }
      };
    generate(noises_vx_, s.sampling_std.vx);
    generate(noises_wz_, s.sampling_std.wz);
    if (is_holonomic_) {
      generate(noises_vy_, s.sampling_std.vy);
    }
    return;
  }

  auto generate = [this](models::BatchArray & target, float standard_deviation) {
      std::normal_distribution<float> distribution(0.0F, standard_deviation);
      target = models::BatchArray::NullaryExpr(
        target.rows(), target.cols(), [this, &distribution]() {return distribution(random_engine_);});
    };

  generate(noises_vx_, s.sampling_std.vx);
  generate(noises_wz_, s.sampling_std.wz);
  if (is_holonomic_) {
    generate(noises_vy_, s.sampling_std.vy);
  }
}

}  // namespace mppi
