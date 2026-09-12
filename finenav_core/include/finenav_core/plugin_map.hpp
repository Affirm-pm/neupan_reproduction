// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

// ══════════════════════════════════════════════════════════════════════════════
//  The MAP author's entry point.
// ══════════════════════════════════════════════════════════════════════════════
//
// Include this in a map plugin — its class header AND its .cpp.  Nothing else
// from finenav_core is needed to write a Map.
//
// The rule, in one sentence: a plugin includes only headers that sit DIRECTLY
// under finenav_core/.  Anything under a subdirectory (common/, map/, plan/,
// worldview/, robot_state/, task_bus/) is framework-internal: it may be split,
// renamed or moved without notice, and it is not part of the plugin contract.
// This file is the contract for the map role — its include list IS the answer
// to "what may a Map rely on?".
//
//     #include "finenav_core/plugin_map.hpp"          // your class + its impl
//     #include "finenav_core/algo_register_macros.hpp" // the registering .cpp
//
// The two are deliberately separate.  Registration instantiates MapServer<MapT>
// and runs the registry, which costs about 3.5 s to compile; these declarations
// cost about 2.0 s.  Keeping them apart pays that 2.0 s per plugin rather than
// the full 3.5 s in every translation unit that includes your map's header.
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
// Everything below is re-exported for the map role:
//
//   common/frames.hpp           FrameNames / frames() — the world-fixed and
//                               robot frame ids for message headers
//                               (default "map" / "base_link")
//   common/geometry_types.hpp   Position3D / Region3D — the geometry vocabulary
//   map/map_concept.hpp         the Map / ShiftableMap concepts
//   map/map_runtime.hpp         MapRuntime — the ONLY framework type a Map
//                               touches, and the one selfManage() is handed.
//                               Re-exported rather than forward-declared so a
//                               map can actually WIRE itself (add an
//                               observation source, a pre/post-update hook)
//                               without reaching into a subdirectory.
//
// A Map never sees MapServer: the server drives the map through this runtime.

#include <rclcpp/rclcpp.hpp>

#include "finenav_core/common/frames.hpp"
#include "finenav_core/common/geometry_types.hpp"
#include "finenav_core/map/map_concept.hpp"
#include "finenav_core/map/map_runtime.hpp"
