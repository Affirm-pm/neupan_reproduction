// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <string>

#include "behaviortree_cpp/decorator_node.h"

namespace finenav::navigator
{

/**
 * @brief A BT::DecoratorNode that ticks its child at a specified rate
 * @note It will re-initialize when halted.
 *
 * Usage in XML:
 * @code
 * <RateController hz="1.0">
 *     <!--Add tree components here-->
 * </RateController>
 * @endcode
 */
class RateController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for finenav::RateController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RateController(
    const std::string & name,
    const BT::NodeConfig & conf);

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("hz", 10.0, "Rate")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time_;
};

}  // namespace finenav::navigator
