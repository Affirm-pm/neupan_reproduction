// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

namespace finenav::core {

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
 *     msg.header.frame_id = finenav::core::frames().world_fixed;
 *
 * This header is re-exported by each finenav_core/plugin_*.hpp entry, so a
 * plugin gets it without naming a subdirectory.
 */
struct FrameNames {
    std::string world_fixed{"map"};  ///< The world-fixed (map) frame.
    std::string robot{"base_link"};  ///< The robot body frame.
};

/**
 * @brief The process-wide frame names.
 *
 * Defined ONCE, out-of-line, in finenav_core.so — the same cross-DSO singleton
 * discipline as the algorithm registries (see FINENAV_DEFINE_REGISTRY_SINGLETON)
 * — so the framework and a plugin DSO cannot disagree about a frame name.
 *
 * This is deliberately NOT an `inline` variable in a header: an inline global
 * would be instantiated once per DSO, and each plugin would silently read its
 * own copy.  That is precisely the failure this exists to prevent.
 *
 * Safe to call from any thread.  The value is written once during setup and only
 * read afterwards.
 */
const FrameNames& frames();

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
}  // namespace finenav::core
