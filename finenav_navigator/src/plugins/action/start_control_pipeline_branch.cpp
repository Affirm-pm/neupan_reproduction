// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_navigator/plugins/action/start_control_pipeline_branch.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include "finenav_navigator/pipeline_controller.hpp"

namespace finenav::navigator {

BT::NodeStatus StartControlPipelineBranch::tick() {
    PipelineController* pipeline = nullptr;
    if (!config().blackboard->get("_pipeline", pipeline) || !pipeline) {
        return BT::NodeStatus::FAILURE;
    }

    std::string to;
    if (!getInput("to", to)) return BT::NodeStatus::FAILURE;

    pipeline->startBranch(to);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace finenav::navigator

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<finenav::navigator::StartControlPipelineBranch>("StartControlPipelineBranch");
}
