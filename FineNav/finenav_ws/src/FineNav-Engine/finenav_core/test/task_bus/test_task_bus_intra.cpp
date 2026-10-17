// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "finenav_core/task_bus/task_bus.hpp"
#include "finenav_interfaces/msg/task_feedback.hpp"

using finenav::core::TaskMaster;
using finenav::core::TaskWorker;
using finenav_interfaces::msg::TaskFeedback;
using finenav_interfaces::msg::Trajectory;

// The task bus must be zero-copy end to end: publishRunning() hands the message
// to the publisher via unique_ptr, and TaskMaster::feedback() returns the same
// shared_ptr.  If anyone reverts the publish to a by-value publish, or disables
// intra-process comms, the received poses vector is a fresh allocation and this
// address assertion fails.
TEST(TaskBusIntra, FeedbackIsZeroCopy)
{
    rclcpp::init(0, nullptr);

    auto node1 = std::make_shared<rclcpp::Node>(
        "task_master_test_node",
        rclcpp::NodeOptions().use_intra_process_comms(true));
    auto node2 = std::make_shared<rclcpp::Node>(
        "task_worker_test_node",
        rclcpp::NodeOptions().use_intra_process_comms(true));

    TaskMaster master(node1);
    TaskWorker worker(node2, "test_layer");
    master.manageLayer("test_layer");

    // IDLE is the structural default: registering the layer seeds the master's
    // latch, so feedback() is non-null (IDLE) before the worker publishes
    // anything — a managed-but-unstarted layer is never "silence".
    {
        auto seeded = master.feedback("test_layer");
        ASSERT_NE(seeded, nullptr);
        EXPECT_EQ(seeded->status, TaskFeedback::IDLE);
    }

    Trajectory traj;
    traj.poses.resize(128);
    for (auto& p : traj.poses) p.position.x = 1.0;
    const auto* sent_ptr = traj.poses.data();

    worker.publishRunning(std::move(traj));

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node1);
    exec.add_node(node2);

    // Wait for the RUNNING to replace the seeded IDLE (a non-null check is no
    // longer enough: the IDLE seed satisfies it immediately).
    auto fb = master.feedback("test_layer");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while ((fb == nullptr || fb->status != TaskFeedback::RUNNING) &&
           std::chrono::steady_clock::now() < deadline) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        fb = master.feedback("test_layer");
    }

    ASSERT_NE(fb, nullptr);
    EXPECT_EQ(fb->status, TaskFeedback::RUNNING);
    ASSERT_EQ(fb->trajectory.poses.size(), 128u);
    EXPECT_EQ(fb->trajectory.poses.data(), sent_ptr);  // decisive zero-copy check

    rclcpp::shutdown();
}
