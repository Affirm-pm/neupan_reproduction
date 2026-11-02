// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_world_view.hpp"

#include "finenav_core/algo_register_macros.hpp"

static_assert(finenav::core::WorldView<finenav::example::DummyLineWorldView, finenav::example::DummyLinePlanner>);
static_assert(finenav::core::WorldView<finenav::example::DummyFollowWorldView, finenav::example::DummyFollowPlanner>);

FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyLineWorldView, dummy_line_world_view,
                           finenav::example::DummyLinePlanner, finenav::example::DummyMap)

FINENAV_REGISTER_WORLDVIEW(finenav::example::DummyFollowWorldView, dummy_follow_world_view,
                           finenav::example::DummyFollowPlanner, finenav::example::DummyMap)
