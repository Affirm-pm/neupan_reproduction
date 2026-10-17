// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>
#include "behaviortree_cpp/control_node.h"

namespace finenav::navigator
{
/**
 * @brief The RecoveryNode has only two children and returns SUCCESS if and only if the first child
 * returns SUCCESS.
 *
 * - If the first child returns FAILURE, the second child will be executed.  After that the first
 * child is executed again if the second child returns SUCCESS.
 *
 * - If the first or second child returns RUNNING, this node returns RUNNING.
 *
 * - If the second child returns FAILURE, this control node will stop the loop and returns FAILURE.
 *
 * Usage in XML:
 * @code
 * <RecoveryNode number_of_retries="1">
 *     <!--Add tree components here-->
 * </RecoveryNode>
 * @endcode
 */
class RecoveryNode : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for finenav::RecoveryNode
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RecoveryNode(
    const std::string & name,
    const BT::NodeConfig & conf);

  /**
   * @brief A destructor for finenav::RecoveryNode
   */
  ~RecoveryNode() override = default;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("number_of_retries", 1, "Number of retries")
    };
  }

private:
  unsigned int current_child_idx_;
  unsigned int number_of_retries_;
  unsigned int retry_count_;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;
};

}  // namespace finenav::navigator
