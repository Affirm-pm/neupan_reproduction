// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <string>

#include "finenav_navigator/plugins/control/round_robin_node.hpp"

namespace finenav::navigator
{

RoundRobinNode::RoundRobinNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::ControlNode(name, config)
{
  getInput("wrap_around", wrap_around_);
}

BT::NodeStatus RoundRobinNode::tick()
{
  const auto num_children = children_nodes_.size();

  setStatus(BT::NodeStatus::RUNNING);
  unsigned num_skipped_children = 0;

  while (num_failed_children_ + num_skipped_children < num_children) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (child_status != BT::NodeStatus::RUNNING) {
      // Increment index and wrap around to the first child if enabled
      if (++current_child_idx_ == num_children) {
        if (wrap_around_) {
          current_child_idx_ = 0;
        } else {
          if (child_status == BT::NodeStatus::SKIPPED) {
            num_skipped_children++;
          } else if (child_status == BT::NodeStatus::FAILURE) {
            num_failed_children_++;
          }
          // Exit early if wrap around is disabled and we've reached the end
          break;
        }
      }
    }

    switch (child_status) {
      case BT::NodeStatus::SUCCESS:
        {
          num_failed_children_ = 0;
          ControlNode::haltChildren();
          return BT::NodeStatus::SUCCESS;
        }

      case BT::NodeStatus::FAILURE:
        {
          num_failed_children_++;
          break;
        }

      case BT::NodeStatus::SKIPPED:
        {
          num_skipped_children++;
          break;
        }
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      default:
        throw BT::LogicError("Invalid status return from BT node");
    }
  }

  const bool all_skipped = (num_skipped_children == num_children);
  halt();
  // If all the children were skipped, this node is considered skipped
  return all_skipped ? BT::NodeStatus::SKIPPED : BT::NodeStatus::FAILURE;
}

void RoundRobinNode::halt()
{
  ControlNode::halt();
  current_child_idx_ = 0;
  num_failed_children_ = 0;
}

}  // namespace finenav::navigator

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<finenav::navigator::RoundRobinNode>("RoundRobin");
}
