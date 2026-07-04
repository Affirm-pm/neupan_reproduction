// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>
#include <shared_mutex>

#include "finenav_core/map/locked_map_view.hpp"
#include "test_helpers.hpp"

using namespace finenav::core;

TEST(LockedMapRO, AccessibleViaOperatorArrow) {
    SmokeMap map;
    std::shared_mutex mtx;

    {
        LockedMapRO<SmokeMap> view(map, mtx);
        EXPECT_TRUE(view->isInside(Position3D::Zero()));
        EXPECT_TRUE(view.get().isInside(Position3D(10, 10, 0)));
    }
    // Lock released, can re-lock:
    {
        LockedMapRO<SmokeMap> view(map, mtx);
        (void)view;
    }
}

TEST(LockedMapRW, AccessibleViaOperatorArrow) {
    SmokeMap map;
    std::shared_mutex mtx;

    {
        LockedMapRW<SmokeMap> view(map, mtx);
        EXPECT_TRUE(view->isInside(Position3D::Zero()));
        view->shiftWindowTo(Position3D(10, 0, 0));
        EXPECT_TRUE(view->isInside(Position3D(10, 0, 0)));
    }
}

TEST(LockedMapRO, NonCopyable) {
    static_assert(!std::is_copy_constructible_v<LockedMapRO<SmokeMap>>);
    static_assert(!std::is_copy_assignable_v<LockedMapRO<SmokeMap>>);
}

TEST(LockedMapRW, NonCopyable) {
    static_assert(!std::is_copy_constructible_v<LockedMapRW<SmokeMap>>);
    static_assert(!std::is_copy_assignable_v<LockedMapRW<SmokeMap>>);
}
