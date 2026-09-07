// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_mapview.hpp"

#include "finenav_core/common/mapview_macros.hpp"

// One-line registration.  The macro records only TYPES (planner + maps); the
// framework resolves those types back to their registered algorithm names
// ("dummy_planner" / "dummy_map") at assembly time — so the MapView author
// never writes a planner/map name by hand, and plugin load order is irrelevant.
FINENAV_REGISTER_MAPVIEW(
    dummy_mapview,
    finenav::example::SimpleMapView,
    finenav::example::DummyPlanner,
    finenav::example::DummyMap)
