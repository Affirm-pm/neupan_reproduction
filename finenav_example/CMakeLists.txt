cmake_minimum_required(VERSION 3.22)
project(finenav_example)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake_auto REQUIRED)

set(finenav_dependencies
        finenav_core
        finenav_localizer
        finenav_navigator
        finenav_motion_decorator
        finenav_interfaces
        finenav_util
)

foreach(dep IN LISTS finenav_dependencies)
    find_package(${dep} REQUIRED)
endforeach()

ament_auto_find_build_dependencies()

add_subdirectory(getting_started)
#add_subdirectory(create_custom_map)
#add_subdirectory(create_custom_planner)

ament_auto_package()
