// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_world_view.hpp"

#include "finenav_core/common/world_view_macros.hpp"

// One-line registration.  The macro records only TYPES (planner + maps); the
// framework resolves those types back to their registered algorithm names
// ("dummy_planner" / "dummy_map") at assembly time — so the WorldView author
// never writes a planner/map name by hand, and plugin load order is irrelevant.
FINENAV_REGISTER_WORLDVIEW(
    dummy_world_view,
    finenav::example::SimpleWorldView,
    finenav::example::DummyPlanner,
    finenav::example::DummyMap)
