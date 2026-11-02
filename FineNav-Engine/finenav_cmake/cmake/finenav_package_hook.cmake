# Copyright (c) 2026.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Included by ament_package() (registered on the `ament_package` extension point
# from finenav_cmake-extras.cmake), i.e. at the very end of the consumer's
# configure — after every add_subdirectory() has had its chance to call
# finenav_register_algo_plugin().
#
# Emits the accumulated manifest ONCE for the whole package.  This is the whole
# reason developers do not have to write an "export" call of their own, and the
# reason the single marker file per package cannot be written twice (a second
# ament_index_register_resource() for the same (type, package) is a hard
# configure error from file(GENERATE)).

# The macro file also declares the constants used below; include it so the hook
# does not depend on having been reached through finenav_cmake-extras.cmake.
include("${finenav_cmake_DIR}/finenav_register_algo_plugin.cmake")

get_property(_finenav_cmake_content
    GLOBAL PROPERTY ${FINENAV_ALGO_PLUGIN_MANIFEST_PROPERTY})

if(NOT _finenav_cmake_content STREQUAL "")
    # CONTENT is passed as ONE argument (it is quoted, and APPEND_STRING kept
    # the `;` list separator out of it), so the newlines inside survive verbatim
    # into the marker file.  file(GENERATE) then expands the
    # $<TARGET_FILE_NAME:...> generator expressions at generate time.
    ament_index_register_resource(
        ${FINENAV_ALGO_PLUGIN_RESOURCE_TYPE} CONTENT "${_finenav_cmake_content}")

    string(REGEX MATCHALL "\n" _finenav_cmake_newlines "${_finenav_cmake_content}")
    list(LENGTH _finenav_cmake_newlines _finenav_cmake_count)
    message(STATUS "finenav_cmake: registered ${_finenav_cmake_count} algorithm plugin(s) "
        "for '${PROJECT_NAME}' in the '${FINENAV_ALGO_PLUGIN_RESOURCE_TYPE}' index")
    unset(_finenav_cmake_newlines)
    unset(_finenav_cmake_count)
endif()

# Clear the accumulator: the manifest for THIS package has been emitted, and
# leaving it set would leak it into a sibling package's marker if several ament
# packages were ever configured inside one CMake process.
set_property(GLOBAL PROPERTY ${FINENAV_ALGO_PLUGIN_MANIFEST_PROPERTY} "")
unset(_finenav_cmake_content)
