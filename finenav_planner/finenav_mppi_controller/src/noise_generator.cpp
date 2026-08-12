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

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

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
    active_ = true;
    ready_ = false;

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

  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  bool notify_worker = false;
  {
    std::lock_guard<std::mutex> guard(noise_lock_);
    settings_ = settings;
    is_holonomic_ = is_holonomic;

    // Recompute the noises on reset, initialization, and fallback.
    xt::noalias(noises_vx_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_vy_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(noises_wz_) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});

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

  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx);
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz);
  if (is_holonomic_) {
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy);
  }
}

}  // namespace mppi
