cmake_minimum_required(VERSION 3.22)
project(finenav_demo)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)

set(dependencies
        finenav_engine
        finenav_map_grid_map
        nav_msgs
        geometry_msgs
)

foreach(dep IN LISTS dependencies)
    find_package(${dep} REQUIRED)
endforeach()

add_subdirectory(stvl_with_mppi)

ament_package()