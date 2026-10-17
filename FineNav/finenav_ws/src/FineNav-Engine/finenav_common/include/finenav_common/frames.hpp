// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

namespace finenav::common {

/**
 * @brief The frame names the framework writes into every message header it
 *        produces.
 *
 * The framework publishes straight from the world-fixed frame to the robot body
 * frame and has no odom frame — by design, and permanently.  So there are
 * exactly two names to agree on, and they have to agree EVERYWHERE: a TF
 * broadcast, an Odometry, a cmd_vel and a visualization marker that disagree
 * produce a disconnected TF tree, which then looks like a localization bug.
 *
 * These are the defaults.  Read them through frames(); override them at startup
 * with FineNavEngine::setFrameNames() when the surrounding system uses different
 * names.
 *
 * A plugin AUTHOR never sets these.  A plugin that needs a frame id reads it:
 *
 *     msg.header.frame_id = finenav::common::frames().world_fixed;
 *
 * This is the only spelling: a plugin writes finenav::common::frames(), and
 * finenav_core deliberately does not mirror it under a second name.  Each
 * finenav_core/plugin_*.hpp entry includes this header, so a plugin reaches it
 * through the include it already has.
 *
 * It lives here, in finenav_common, rather than in finenav_core, because
 * finenav_visualizer stamps frame ids and sits BELOW finenav_core: a frame name
 * is a fact about the messages the framework produces, not a fact about
 * planning, and the publisher has to be able to see it.  finenav_common is the
 * one layer both of them sit on.
 */
struct FrameNames {
    std::string world_fixed{"map"};  ///< The world-fixed (map) frame.
    std::string robot{"base_link"};  ///< The robot body frame.
};

/**
 * @brief Which of the two frames a value belongs to.
 *
 * A frame id is a string, and a string is a thing a caller can get wrong:
 * setFrame("map") and setFrame("odom") both compile, and the second one draws
 * into a frame the framework never publishes, which then reads as a
 * localization bug.  So the visualizer takes THIS rather than a name — a tag
 * cannot be misspelled — and resolves it through frames() at publish time.
 *
 * Resolving late is the point.  The names are a runtime choice of the
 * surrounding system (see FineNavEngine::setFrameNames), so Frame is a tag and
 * never a copy of the string: a channel holding Frame::World stamps whatever
 * frames().world_fixed says at the moment it publishes.
 */
enum class Frame {
    World,  ///< frames().world_fixed — a global path, a costmap, a goal.
    Robot,  ///< frames().robot — a footprint, a velocity arrow, a sensor axis.
};

/**
 * @brief The process-wide frame names.
 *
 * Defined ONCE, out-of-line, in finenav_common.so — by hand here, because a
 * FrameNames is not a registry table, but under exactly the discipline
 * FINENAV_DEFINE_REGISTRY_SINGLETON applies to the algorithm registries
 * (finenav_common/detail/named_registry.hpp): one non-inline definition in one
 * shared object, so the framework, the visualizer and a plugin DSO cannot
 * disagree about a frame name.
 *
 * This is deliberately NOT an `inline` variable in a header: an inline global
 * would be instantiated once per DSO, and each plugin would silently read its
 * own copy.  That is precisely the failure this exists to prevent.
 *
 * Safe to call from any thread.  The value is written once during setup and only
 * read afterwards.
 */
const FrameNames& frames();

/**
 * @brief The name behind @p frame, as frames() has it right now.
 *
 * The returned reference is into the process-wide names, so it is valid for as
 * long as the process is — a caller may hold it, and it never dangles.
 */
const std::string& frameName(Frame frame);

namespace detail {

/**
 * @brief Override the process-wide frame names.  FRAMEWORK-INTERNAL.
 *
 * Called by FineNavEngine, never by a plugin: a plugin READS frames(), it does
 * not set it.  Prefer FineNavEngine::setFrameNames(), which additionally
 * enforces the "during setup, before anything is assembled" ordering.
 *
 * Set-once.  A second call is ignored and logged as an ERROR, because a frame id
 * changed after the fact silently invalidates every message already published
 * under the old name.
 *
 * @return false if the names had already been set (this call was ignored).
 */
bool setFrameNames(FrameNames names);

}  // namespace detail
}  // namespace finenav::common
