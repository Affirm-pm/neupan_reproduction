cmake_minimum_required(VERSION 3.22)
project(finenav_demo)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

add_subdirectory(stvl_with_mppi)

ament_auto_package()