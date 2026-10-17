// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <chrono>
#include <string>

#include "finenav_navigator/plugins/decorator/rate_controller.hpp"

namespace finenav::navigator
{

RateController::RateController(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::DecoratorNode(name, conf),
  first_time_(false)
{
}

void RateController::initialize()
{
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
}

BT::NodeStatus RateController::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
    // Reset the starting point since we're starting a new iteration of
    // the rate controller (moving from IDLE to RUNNING)
    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  // Determine how long its been since we've started this iteration
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  // Now, get that in seconds
  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    seconds.count() >= period_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::SKIPPED:
      case BT::NodeStatus::RUNNING:
      case BT::NodeStatus::FAILURE:
        return child_state;

      case BT::NodeStatus::SUCCESS:
        start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
        return BT::NodeStatus::SUCCESS;

      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace finenav::navigator

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<finenav::navigator::RateController>("RateController");
}
