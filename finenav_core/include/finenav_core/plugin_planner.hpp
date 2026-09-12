// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// ══════════════════════════════════════════════════════════════════════════════
//  The PLANNER author's entry point.
// ══════════════════════════════════════════════════════════════════════════════
//
// Include this in a planner plugin — its class header AND its .cpp.  Nothing
// else from finenav_core is needed to write a Planner.
//
// The rule, in one sentence: a plugin includes only headers that sit DIRECTLY
// under finenav_core/.  Anything under a subdirectory (common/, map/, plan/,
// worldview/, robot_state/, task_bus/) is framework-internal: it may be split,
// renamed or moved without notice, and it is not part of the plugin contract.
// This file is the contract for the planner role — its include list IS the
// answer to "what may a Planner rely on?".
//
//     #include "finenav_core/plugin_planner.hpp"        // your class + its impl
//     #include "finenav_core/algo_register_macros.hpp"  // the registering .cpp
//
// The two are deliberately separate.  Registration instantiates
// ControlLayer<PlannerT> and runs the registry, which costs about 3.5 s to
// compile; these declarations cost about 1.9 s.  Keeping them apart pays that
// 1.9 s per plugin rather than the full 3.5 s in every translation unit that
// includes your planner's header.
//
// rclcpp is part of the contract for all three roles, and it is included here
// explicitly rather than left to arrive by accident.  A plugin's own optional
// conventions name rclcpp types — setLogger(rclcpp::Logger) — so it must be
// there for a plugin that declares them; and it must NOT be something a plugin
// only gets because it happened to include the visualizer (see below).
//
// Nothing in a plugin's own include list costs more than this unless it opts
// into visualization: finenav_visualizer.hpp alone is about 3.3 s (rclcpp +
// PCL + visualization_msgs), and declaring setVisualizer() is OPTIONAL.  A
// plugin that does not visualize should not include it.
//
// Everything below is re-exported for the planner role:
//
//   common/geometry_types.hpp   Position3D / Region3D — the geometry vocabulary
//   plan/planner_concept.hpp    the Planner concept, PlanningContext, and (via
//                               plan/trajectory.hpp) Trajectory / IOProfile /
//                               InputTrajectoryT / OutputTrajectoryT
//
// A planner declares its own IMapView; it is handed one, never a Map.  That is
// why nothing from map/ appears here.

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/common/geometry_types.hpp"
#include "finenav_core/plan/planner_concept.hpp"
