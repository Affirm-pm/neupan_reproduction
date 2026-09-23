cmake_minimum_required(VERSION 3.22)
project(finenav_common)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

# A shared library, not a header-only interface, and deliberately so: this is
# where the ONE definition of frames() lives.  A header-only frames() would be
# instantiated once per DSO and each plugin would read its own copy — the exact
# failure the cross-DSO singleton discipline exists to prevent.  See
# include/finenav_common/frames.hpp.
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/frame_names.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
