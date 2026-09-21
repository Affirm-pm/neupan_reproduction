// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// ControlLayer fault propagation over the REAL TaskMaster ↔ TaskWorker channel.
//
// Regression guard for commit 4ca5717.  An EventDriven ControlLayer whose
// plan() returned std::nullopt used to drop the failure silently: react() ran on
// the subscription callback thread and simply returned without publishing
// anything, so the master — and every downstream stage consuming the layer's
// __feedback — kept seeing a frozen RUNNING reference.  Two properties are
// asserted, both through the public TaskMaster surface rather than by poking
// layer internals:
//
//   1. a failing plan() publishes FAULTED — not silence;
//   2. FAULTED is terminal: a later STOP must NOT overwrite it with STOPPED.
//
// Both policies are covered, because both share the episode-body fault path.
// The layer is driven exactly as the engine drives it at runtime: the master
// node and the layer's private node are built with the SAME namespace (the
// engine node's name, which every private node takes as its namespace — see
// PipelineNode's constructor), so TaskChannel's relative topic names resolve to
// one absolute pair — /fault_test_engine/<layer>/__cmd and .../__feedback.

#include <atomic>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <variant>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>

#include "finenav_core/plan/control_layer.hpp"
#include "finenav_core/plan/execution_policy.hpp"
#include "finenav_core/task_bus/task_bus.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"
#include "finenav_interfaces/msg/trajectory.hpp"

#include "test_helpers.hpp"  // TestProvider / makeState (finenav_core/test/robot_state)

using finenav_interfaces::msg::TaskFeedback;
using finenav_interfaces::msg::Trajectory;

namespace {

// ── Scenario constants ────────────────────────────────────────────────────────
// PipelineNode builds its private node as
//   make_shared(layer_name, engine_node->get_name(), opts)
// so the ENGINE NODE NAME doubles as the namespace of every private node.  The
// master must be built the same way (BtNavigator does exactly this) or the two
// sides land on different absolute topics.
constexpr const char* kEngineNodeName  = "fault_test_engine";
constexpr const char* kMasterNodeName  = "finenav_navigator";
constexpr const char* kUpstreamChannel = "upstream";
constexpr const char* kPlannerName     = "test_fault_planner";

// ── A map-less planner that always fails ─────────────────────────────────────
// No IWorldView → the map-less branch of the Planner concept, so the layer needs
// no WorldView attached.  No FINENAV_REGISTER_PLANNER entry either: an
// unregistered algo name only makes the constructor log a warning.
struct FaultPlannerConfig {
    double dummy{0.0};
};

class FaultPlanner {
public:
    using ConfigType = FaultPlannerConfig;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    void configure(const FaultPlannerConfig&) {}
    void reset() {}

    std::optional<finenav::core::OutputTrajectoryT<FaultPlanner>>
    plan(const finenav::core::PlanningContext<FaultPlanner>&) {
        plan_calls.fetch_add(1, std::memory_order_relaxed);
        return std::nullopt;  // the failure under test
    }

    /// How many times plan() actually ran.  Asserting this is 1 proves the
    /// FAULTED came from the plan() path, not from the waitForState() path
    /// (startWorker() faults on that too when the state provider is empty).
    static std::atomic<int> plan_calls;
};

std::atomic<int> FaultPlanner::plan_calls{0};

/// ControlLayer with the inherited TaskWorker promoted, so the test can prove
/// the worker → master feedback path is live BEFORE the first START.
class FaultLayer : public finenav::core::ControlLayer<FaultPlanner> {
public:
    using finenav::core::ControlLayer<FaultPlanner>::ControlLayer;

    void publishIdleProbe() { this->worker_->publishIdle(); }
};

// ── Helpers ──────────────────────────────────────────────────────────────────

/// The reference the layer consumes: a POSE-only trajectory, because
/// planOnce() → detail::fromROSMsg() throws when a profile's fields are absent.
Trajectory makeReference() {
    Trajectory traj;
    traj.valid_fields = Trajectory::TRAJ_POSE;
    traj.poses.resize(2);
    traj.poses.back().position.x = 2.0;
    return traj;
}

using Executor = rclcpp::executors::SingleThreadedExecutor;

/// Spin `exec` (1 ms poll) until `done()` or `timeout` elapses.
template <typename Pred>
bool spinUntil(Executor& exec, Pred done, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
        if (done()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    exec.spin_some();
    return done();
}

void spinFor(Executor& exec, std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

struct FaultOutcome {
    bool    cmd_sub_seen{false};      ///< the master's node saw the worker's command subscription
    bool    faulted{false};           ///< FAULTED latched by the master while the layer was STARTed
    uint8_t status_after_stop{0};     ///< status latched after a subsequent STOP
    int     plan_calls{0};            ///< plan() invocations observed
};

/// Drive one full episode: wire master → layer → upstream, START, wait for the
/// FAULTED to reach the master, then STOP and re-read.
FaultOutcome runFaultScenario(const std::string& layer_name,
                              finenav::core::ExecutionPolicyVariant policy)
{
    FaultOutcome out;

    auto opts = rclcpp::NodeOptions().use_intra_process_comms(true);

    auto engine_node   = rclcpp::Node::make_shared(kEngineNodeName, opts);
    // Same construction as BtNavigator's private node: the parent node's NAME is
    // the namespace, so master and worker share /fault_test_engine.
    auto master_node   = rclcpp::Node::make_shared(kMasterNodeName, engine_node->get_name(), opts);
    auto upstream_node = rclcpp::Node::make_shared("upstream_worker", engine_node->get_name(), opts);

    Executor exec;
    exec.add_node(master_node);
    exec.add_node(upstream_node);

    finenav::core::TaskMaster master(master_node);
    finenav::core::TaskWorker upstream(upstream_node, kUpstreamChannel);

    TestProvider provider;
    provider.pushState(makeState(0.0, 0.0, 0.0));  // so waitForState() succeeds

    // Inner scope: the layer (with its pnode + NodeThread + worker thread) is
    // destroyed while the master and its executor are still alive.
    {
        FaultPlanner::plan_calls.store(0);
        FaultLayer layer(engine_node, kPlannerName, layer_name, std::move(policy), &provider);
        layer.connectFrom(kUpstreamChannel);
        master.manageLayer(layer_name);

        // ── Readiness 1: the feedback path (worker → master) is live.  The
        //    worker's constructor IDLE may predate the master's subscription, so
        //    re-publish it and wait for the master to latch it.
        layer.publishIdleProbe();
        spinUntil(exec, [&] { return master.feedback(layer_name) != nullptr; },
                  std::chrono::milliseconds(2000));

        // ── Readiness 2: feed the reference BEFORE START.  The event-driven body
        //    consumes a latched reference; the periodic body needs one to leave
        //    kNoInput (it rate-sleeps forever otherwise).  Published repeatedly
        //    because the layer's subscription was created moments ago.
        for (int i = 0; i < 5; ++i) {
            upstream.publishRunning(makeReference());
            spinFor(exec, std::chrono::milliseconds(50));
        }

        // The command path (master → worker) must be matched before STARTing: a
        // START published before the worker's subscription exists is lost, and
        // the layer would then never run (a flaky, misleading failure).
        const auto pnode = layer.getPrivateNode();
        const std::string cmd_topic = rclcpp::expand_topic_or_service_name(
            finenav::core::TaskChannel::cmdTopic(layer_name),
            pnode->get_name(), pnode->get_namespace());
        out.cmd_sub_seen = spinUntil(
            exec, [&] { return master_node->count_subscribers(cmd_topic) > 0; },
            std::chrono::milliseconds(3000));

        // ── START: the layer plans on its worker thread and must FAULT.
        master.start(layer_name);
        out.faulted = spinUntil(exec, [&] {
            auto fb = master.feedback(layer_name);
            return fb != nullptr && fb->status == TaskFeedback::FAULTED;
        }, std::chrono::seconds(5));

        // ── STOP: FAULTED is terminal.  The STOP may not resurrect the layer and
        //    overwrite the fault with STOPPED (the pre-4ca5717 behaviour).
        master.stop(layer_name);
        spinFor(exec, std::chrono::milliseconds(300));

        auto fb = master.feedback(layer_name);
        out.status_after_stop = fb ? fb->status : 255;  // 255 = still silence
        out.plan_calls = FaultPlanner::plan_calls.load();
    }

    return out;
}

}  // namespace

TEST(ControlLayerFault, EventDrivenPlanFailurePublishesTerminalFault)
{
    rclcpp::init(0, nullptr);
    const auto out = runFaultScenario("l_event", finenav::core::EventDrivenPolicy{});

    EXPECT_TRUE(out.cmd_sub_seen)
        << "the master's node never saw the worker's command subscription on "
           "l_event/__cmd — TaskMaster::start() could not reach the layer";
    EXPECT_EQ(out.plan_calls, 1)
        << "plan() did not run exactly once on the worker thread";
    EXPECT_TRUE(out.faulted)
        << "no FAULTED ever reached the master: a failing plan() must not fail silently";
    EXPECT_EQ(static_cast<int>(out.status_after_stop), static_cast<int>(TaskFeedback::FAULTED))
        << "STOP overwrote the FAULTED latch (status after STOP = "
        << static_cast<int>(out.status_after_stop) << ")";

    rclcpp::shutdown();
}

TEST(ControlLayerFault, PeriodicPlanFailurePublishesTerminalFault)
{
    rclcpp::init(0, nullptr);
    const auto out = runFaultScenario("l_periodic", finenav::core::PeriodicPolicy{10.0});

    EXPECT_TRUE(out.cmd_sub_seen)
        << "the master's node never saw the worker's command subscription on "
           "l_periodic/__cmd — TaskMaster::start() could not reach the layer";
    EXPECT_EQ(out.plan_calls, 1)
        << "plan() did not run exactly once on the worker thread";
    EXPECT_TRUE(out.faulted)
        << "no FAULTED ever reached the master: a failing plan() must not fail silently";
    EXPECT_EQ(static_cast<int>(out.status_after_stop), static_cast<int>(TaskFeedback::FAULTED))
        << "STOP overwrote the FAULTED latch (status after STOP = "
        << static_cast<int>(out.status_after_stop) << ")";

    rclcpp::shutdown();
}
