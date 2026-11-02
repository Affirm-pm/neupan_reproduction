# Copyright (c) 2026.
# IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
# All rights reserved.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Included automatically by finenav_cmakeConfig.cmake (ament_package installed
# it as a CONFIG_EXTRAS), i.e. at the exact point a consumer writes
# `find_package(finenav_cmake REQUIRED)`.
#
# Both things done here MUST land in the *consumer's* scope, which is why they
# happen in this file rather than in finenav_cmake's own CMakeLists:
# ament_register_extension() appends to the AMENT_EXTENSIONS_ament_package
# variable, which ament_execute_extensions() then reads from that same scope
# when the consumer calls ament_package().  This is pluginlib's and
# rclcpp_components' wiring, reproduced.

find_package(ament_cmake_core QUIET REQUIRED)

# Bring in finenav_register_algo_plugin() (and the constants it declares).
include("${finenav_cmake_DIR}/finenav_register_algo_plugin.cmake")

# Hook the manifest emission onto the consumer's ament_package() call.  Guarded
# against a second find_package(finenav_cmake) in the same scope appending the
# hook twice.
set(_finenav_cmake_hook "finenav_cmake:finenav_package_hook.cmake")
if(NOT _finenav_cmake_hook IN_LIST AMENT_EXTENSIONS_ament_package)
    ament_register_extension("ament_package" "finenav_cmake"
        "finenav_package_hook.cmake")
endif()
unset(_finenav_cmake_hook)
