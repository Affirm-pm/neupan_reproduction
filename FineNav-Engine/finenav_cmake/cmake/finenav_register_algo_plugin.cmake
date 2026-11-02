# Copyright (c) 2026.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

if(FINENAV_REGISTER_ALGO_PLUGIN_CMAKE_INCLUDED)
    return()
endif()
set(FINENAV_REGISTER_ALGO_PLUGIN_CMAKE_INCLUDED TRUE)

# ─────────────────────────────────────────────────────────────────────────────
# finenav_register_algo_plugin(<target>)
#
# WHAT A PLUGIN DEVELOPER WRITES
#
#   find_package(finenav_cmake REQUIRED)          # once, in the package root
#   add_library(dummy_planner SHARED src/dummy_planner.cpp)
#   install(TARGETS dummy_planner LIBRARY DESTINATION lib)
#   finenav_register_algo_plugin(dummy_planner)   # ← right next to the target
#
# That one call is the whole interface.  It can be written in ANY subdirectory
# of the package (typically the one that defines the library), any number of
# times, in any order — the calls aggregate automatically.
#
# WHY IT EXISTS
#
# The engine discovers algorithm plugins (planner / map / world view) by
# dlopen-ing their .so files: loading is what fires the static self-registrars
# that populate the shared PlanRegistry / MapRegistry / WorldViewRegistry.  To
# find them it reads an ament resource index marker for the package — one line
# per plugin .so, relative to the install prefix.  This macro is what writes
# that marker.
#
# The goal is that an *application* author writes only
#
#   engine->loadAlgoPlugins({"finenav_example"});   // a bare package name
#
# and never learns where the .so files live.  Everything install-layout-shaped
# therefore has to be decided here, once, by the plugin package:
#
#   1. the `finenav_algo_plugins` resource-type literal (engine + every plugin
#      package would otherwise spell it independently, and a typo would drop
#      plugins silently at runtime);
#   2. the `lib/` install sub-directory (otherwise the developer repeats, in
#      `install(... DESTINATION lib)` and in the manifest, a string that must
#      match but has no mechanical link — see FINENAV_ALGO_PLUGIN_LIBDIR);
#   3. the `$<TARGET_FILE_NAME:...>` generator expression, so the manifest
#      cannot drift from the install() rule when a library is renamed.
#
# HOW IT AGGREGATES
#
# The target name is appended to a GLOBAL property, which is process-wide and
# therefore survives the CMake directory scopes that a plain `set()` would be
# trapped in.  The accumulated string is emitted ONCE, at the end of the
# consumer's configure, by cmake/finenav_package_hook.cmake — registered on the
# `ament_package` extension point by cmake/finenav_cmake-extras.cmake.  So the
# developer never writes a separate "export" call, and the single marker file
# per package can never be written twice (calling ament_index_register_resource
# twice for the same (type, package) is a hard configure error).
# ─────────────────────────────────────────────────────────────────────────────

# ── The one place these literals are spelled ────────────────────────────────
# The ament resource type the engine reads.  Renaming the resource is a change
# to this line plus the engine's matching constant — nothing else.
set(FINENAV_ALGO_PLUGIN_RESOURCE_TYPE "finenav_algo_plugins")

# Name of the GLOBAL property holding the not-yet-emitted manifest.  The
# property lives in the configure process, so it is invisible to other packages.
set(FINENAV_ALGO_PLUGIN_MANIFEST_PROPERTY "FINENAV_ALGO_PLUGIN_MANIFEST_CONTENT")

# Install sub-directory the manifest paths are written relative to.  This must
# match the `LIBRARY DESTINATION` the package installs its plugin libraries to
# (the ROS convention, and what this macro assumes).  A package that installs
# its plugins somewhere else has to change it before calling the macro:
#   set(FINENAV_ALGO_PLUGIN_LIBDIR "lib64")
set(FINENAV_ALGO_PLUGIN_LIBDIR "lib")


# Register `target` as a discoverable FineNav algorithm plugin.
#
# This only *registers an already-built library for discovery*.  It deliberately
# does NOT wrap add_library() or install(): the developer keeps full control of
# SHARED vs STATIC, sources, DEFINES, include directories and install rules — a
# one-size-fits-all macro would have to assume all of them.
#
# :param target: the library target to publish into the ament resource index
#   (must already exist; this macro does not create it)
# :type target: string
#
# @public
#
function(finenav_register_algo_plugin target)
    if(NOT TARGET ${target})
        message(FATAL_ERROR
            "finenav_register_algo_plugin(${target}): '${target}' is not a CMake target. "
            "This macro registers an existing library for discovery — call "
            "add_library(${target} ...) before it.")
    endif()

    # APPEND_STRING and NOT APPEND, deliberately.  A CMake property is a plain
    # string, but APPEND joins the appended value to what is already there with
    # `;` — CMake's list separator.  That `;` would land in the middle of a
    # manifest line ("…/lib/;libdummy_map.so"), and the engine would look for a
    # file with a `;` in its name, warn, and silently skip the plugin.
    # APPEND_STRING is a raw string concatenation, so no separator is inserted.
    set_property(GLOBAL
        APPEND_STRING
        PROPERTY ${FINENAV_ALGO_PLUGIN_MANIFEST_PROPERTY}
        "${FINENAV_ALGO_PLUGIN_LIBDIR}/$<TARGET_FILE_NAME:${target}>\n")
endfunction()
