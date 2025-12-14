cmake_minimum_required(VERSION 3.22)
project(finenav_interface)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_planner_node src/local_layer_node.cpp)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_auto_package()