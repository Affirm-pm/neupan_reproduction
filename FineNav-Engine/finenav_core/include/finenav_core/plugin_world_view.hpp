// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// ══════════════════════════════════════════════════════════════════════════════
//  The WORLD VIEW author's entry point.
// ══════════════════════════════════════════════════════════════════════════════
//
// Include this in a WorldView plugin — the one artifact that knows both a
// concrete Map and a concrete Planner.  Nothing else from finenav_core is
// needed to write one.
//
// The rule, in one sentence: a plugin includes only headers that sit DIRECTLY
// under finenav_core/.  Anything under a subdirectory (common/, map/, plan/,
// worldview/, robot_state/, task_bus/) is framework-internal: it may be split,
// renamed or moved without notice, and it is not part of the plugin contract.
// This file is the contract for the WorldView role — its include list IS the
// answer to "what may a WorldView rely on?".
//
//     #include "finenav_core/plugin_world_view.hpp"     // your view
//     #include "finenav_core/algo_register_macros.hpp"   // the registering .cpp
//
// The two are deliberately separate.  Registration instantiates ControlLayer,
// MapServer and the registries, which costs about 3.5 s to compile; these
// declarations cost about 1.9 s.  Keeping them apart pays that 1.9 s per plugin
// rather than the full 3.5 s in every translation unit that includes your
// view's header.
//
// rclcpp is part of the contract for all three roles, and it is included here
// explicitly rather than left to arrive by accident.  A plugin's own optional
// conventions name rclcpp types — setLogger(rclcpp::Logger), and a WorldView's
// constructor takes rclcpp::Node::SharedPtr — so it must be there for a plugin
// that declares them; and it must NOT be something a plugin only gets because
// it happened to include the visualizer (see below).
//
// Nothing in a plugin's own include list costs more than this unless it opts
// into visualization: finenav_visualizer.hpp alone is about 3.3 s (rclcpp +
// PCL + visualization_msgs), and declaring setVisualizer() is OPTIONAL.  A
// plugin that does not visualize should not include it.
//
// Everything below is what a WorldView is given, and it comes from two layers:
//
//   finenav_common/frames.hpp          FrameNames / Frame / frames() — the
//                                      world-fixed and robot frame ids for
//                                      message headers, and the tag a marker is
//                                      stamped with
//   finenav_common/geometry_types.hpp  Position3D / Region3D
//   worldview/world_view_concept.hpp   the WorldView concept
//
// One rule covers both layers: the VOCABULARY — the shared types whose
// definition precedes finenav_core, so every role and the visualizer can name
// them — is spelled finenav::common::; the ROLE interface is spelled
// finenav::core::.  Neither has a second name: a type with two spellings is a
// coin-flip a plugin author should never have to make.
//
// Note the dependency direction: world_view_concept.hpp includes
// plan/planner_concept.hpp, so this file is a strict SUPERSET of
// plugin_planner.hpp — a WorldView author gets PlanningContext / Trajectory /
// IOProfile too, because prepare() takes a PlanningContext<PlannerT> and the
// view implements its planner's IWorldView.  There is deliberately no way to
// take a WorldView entry that excludes the planner vocabulary; the three role
// entries are nested (map ⊂ planner ⊆ world_view), not three independent
// drawers.
//
// A WorldView is handed the maps at construction and reads them directly; it
// is the only artifact that sees both sides of the world/robot line.

#include <rclcpp/rclcpp.hpp>

#include "finenav_common/frames.hpp"
#include "finenav_common/geometry_types.hpp"
#include "finenav_core/worldview/world_view_concept.hpp"
