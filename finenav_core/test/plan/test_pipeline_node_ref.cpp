// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// Episode-boundary reference hygiene on PipelineNode.
//
// A downstream layer that restarts re-reads its upstream's still-latched
// FAULTED reference and re-faults before the upstream has produced fresh
// output.  clearFaultedPlanRef() breaks that race: at every episode end
// (publishFaulted / publishStopped) the stashed reference is dropped IF it is
// FAULTED, and kept if it is RUNNING — a RUNNING reference is the current
// valid input (the navigation goal for the first stage, or the upstream's live
// output), not stale state.

#include <memory>
#include <optional>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "finenav_core/plan/pipeline_node.hpp"
#include "finenav_core/plugin_planner.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"

using finenav::core::PlanRefState;
using finenav_interfaces::msg::TaskFeedback;

namespace {

// ── Minimal map-less test planner ─────────────────────────────────────────
// No IWorldView → the map-less branch of the Planner concept, so PipelineNode
// needs no WorldView attached.  No FINENAV_REGISTER_PLANNER entry either: an
// unregistered algo name only makes the constructor log a warning.
struct TestPlannerConfig {
    double dummy{0.0};
};

class TestPlanner {
public:
    using ConfigType = TestPlannerConfig;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    void configure(const TestPlannerConfig&) {}
    void reset() {}

    std::optional<finenav::core::OutputTrajectoryT<TestPlanner>>
    plan(const finenav::core::PlanningContext<TestPlanner>& ctx) {
        finenav::core::OutputTrajectoryT<TestPlanner> out;
        out.poses = ctx.ref_traj.poses;
        return out;
    }
};

// Exposes the protected reference-state surface under test.
class TestPipelineNode : public finenav::core::PipelineNode<TestPlanner> {
public:
    TestPipelineNode(rclcpp::Node::SharedPtr node, const std::string& layer_name)
        : PipelineNode<TestPlanner>(node, "test_planner", layer_name, nullptr) {}

    using PipelineNode<TestPlanner>::storePlanRef;
    using PipelineNode<TestPlanner>::planRefState;
    using PipelineNode<TestPlanner>::clearFaultedPlanRef;
};

std::shared_ptr<TaskFeedback> makeFeedback(uint8_t status) {
    auto msg = std::make_shared<TaskFeedback>();
    msg->status = status;
    return msg;
}

}  // namespace

TEST(PipelineNodeRef, ClearFaultedPlanRefSemantics)
{
    rclcpp::init(0, nullptr);
    {
        auto node = std::make_shared<rclcpp::Node>(
            "pipeline_node_ref_test",
            rclcpp::NodeOptions().use_intra_process_comms(true));
        TestPipelineNode pnode(node, "ref_test_layer");

        // 1) No reference stored: the helper must be a harmless no-op.
        ASSERT_EQ(pnode.planRefState(), PlanRefState::kNoInput);
        pnode.clearFaultedPlanRef();
        EXPECT_EQ(pnode.planRefState(), PlanRefState::kNoInput);

        // 2) A FAULTED reference is cleared → back to "no input", so a
        //    restarted layer waits for fresh upstream output.
        pnode.storePlanRef(makeFeedback(TaskFeedback::FAULTED));
        ASSERT_EQ(pnode.planRefState(), PlanRefState::kUpstreamFaulted);
        pnode.clearFaultedPlanRef();
        EXPECT_EQ(pnode.planRefState(), PlanRefState::kNoInput);

        // 3) A RUNNING reference is the current valid input and MUST survive.
        pnode.storePlanRef(makeFeedback(TaskFeedback::RUNNING));
        ASSERT_EQ(pnode.planRefState(), PlanRefState::kValid);
        pnode.clearFaultedPlanRef();
        EXPECT_EQ(pnode.planRefState(), PlanRefState::kValid);
    }
    rclcpp::shutdown();
}
