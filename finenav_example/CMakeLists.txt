cmake_minimum_required(VERSION 3.22)
project(finenav_example)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(finenav_engine REQUIRED)

ament_auto_find_build_dependencies()

add_subdirectory(create_custom_map)
add_subdirectory(create_custom_planner)
add_subdirectory(getting_started)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
