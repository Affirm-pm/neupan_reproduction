// Copyright (c) 2026.bt_action_node.hpp
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "behavior_tree_engine.hpp"

#include "finenav_util/simple_action_server.hpp"
#include "finenav_msgs/action/navigate_to_pose.hpp"

#include "finenav_navigator/navigator_params.hpp"

namespace finenav {

class Navigator : public rclcpp::Node {
  public:

    using ActionT = finenav_msgs::action::NavigateToPose; // TODO: 改成用户发送目标的 action
    using ActionServer = SimpleActionServer<ActionT>;

    explicit Navigator(const rclcpp::NodeOptions & options);
    virtual ~Navigator() = default;

    void initialize();


  protected:
    void execute();


  private:
    std::unique_ptr<BehaviorTreeEngine> bt_engine_;
    std::unique_ptr<ActionServer> action_server_;

    finenav_navigator::Params params_;
    std::unique_ptr<finenav_navigator::ParamListener> param_listener_;

    BT::Tree tree_;

};

}