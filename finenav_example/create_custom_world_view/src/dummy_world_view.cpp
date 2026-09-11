// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "dummy_world_view.hpp"

#include "finenav_core/common/world_view_macros.hpp"

namespace finenav::example {

void SimpleWorldView::prepare(const finenav::core::PlanningContext<DummyPlanner>& ctx) {
    // Real heavy derivation (inflation, terrain fields, cost maps) would run
    // here, under the map read lock, exactly once per plan().  This view has
    // nothing to derive, so it just proves the hook fires.
    ++prepare_count_;
    if (prepare_log_period_ <= 0 || prepare_count_ % prepare_log_period_ != 0) {
        return;
    }
    const auto& p = ctx.robot_state.pose.position;
    RCLCPP_INFO(logger_, "SimpleWorldView::prepare() #%ld — robot at (%.2f, %.2f).",
        static_cast<long>(prepare_count_), p.x, p.y);
}

}  // namespace finenav::example

// One-line registration.  The macro records only TYPES (planner + maps); the
// framework resolves those types back to their registered algorithm names
// ("dummy_planner" / "dummy_map") at assembly time — so the WorldView author
// never writes a planner/map name by hand, and plugin load order is irrelevant.
//
// Arguments: (view type, param namespace, planner type, maps...).  As with
// FINENAV_REGISTER_PLANNER / _MAP, that one namespace token is also the
// registration name — so it must match the top-level key of
// dummy_world_view.yaml.
FINENAV_REGISTER_WORLDVIEW(
    finenav::example::SimpleWorldView,
    dummy_world_view,
    finenav::example::DummyPlanner,
    finenav::example::DummyMap)
