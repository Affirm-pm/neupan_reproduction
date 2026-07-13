// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "finenav_localizer/observation_queue.hpp"

using namespace finenav::localizer;

// A trivial value type for the queue (no real ROS message).
using IntQueue = AgedObservationQueue<int>;

// ── Push / pop FIFO order ─────────────────────────────────────────────────────
TEST(AgedObservationQueue, PushPop_InFIFOOrder)
{
    IntQueue q(2);   // max_age = 2 (each item survives at most 2 pops)
    q.push(1);
    q.push(2);
    EXPECT_EQ(q.size(), 2u);

    const int a = q.pop_increment_age();
    const int b = q.pop_increment_age();
    EXPECT_EQ(a, 1);
    EXPECT_EQ(b, 2);
    // items were re-enqueued at age=1 (still < max_age=2), so they stay.
    EXPECT_EQ(q.size(), 2u);
}

// ── Aging: after max_age pops the item is dropped ──────────────────────────────
TEST(AgedObservationQueue, Aging_DroppedAfterMaxAge)
{
    IntQueue q(1);   // max_age = 1 → survives 0 re-queues (immediate drop)
    q.push(42);
    EXPECT_EQ(q.size(), 1u);
    q.pop_increment_age();   // age becomes 1 → >= max_age → dropped
    EXPECT_TRUE(q.empty());
}

// ── Aging: exactly at max_age − 1 it re-enqueues ───────────────────────────────
TEST(AgedObservationQueue, Aging_StaysUntilMaxAge)
{
    IntQueue q(3);
    q.push(10);
    q.pop_increment_age(); // age=1 < 3 → re-enqueue → size stays 1
    EXPECT_EQ(q.size(), 1u);
    q.pop_increment_age(); // age=2 < 3 → re-enqueue
    EXPECT_EQ(q.size(), 1u);
    q.pop_increment_age(); // age=3 >= 3 → drop
    EXPECT_TRUE(q.empty());
}

// ── Capacity overflow ──────────────────────────────────────────────────────────
TEST(AgedObservationQueue, CapacityOverflow_DropsOldest)
{
    IntQueue q(1, 2);  // max_age=1, max_queue_size=2
    q.push(1); q.push(2); q.push(3);  // overflow: oldest (1) dropped
    EXPECT_EQ(q.size(), 2u);
    EXPECT_EQ(q.pop_increment_age(), 2);
    EXPECT_EQ(q.pop_increment_age(), 3);
}

// ── Clear ──────────────────────────────────────────────────────────────────────
TEST(AgedObservationQueue, Clear_Empties)
{
    IntQueue q(2);
    q.push(7); q.push(8);
    q.clear();
    EXPECT_TRUE(q.empty());
    EXPECT_EQ(q.size(), 0u);
}
