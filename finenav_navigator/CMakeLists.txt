cmake_minimum_required(VERSION 3.22)
project(finenav_navigator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_library(test_library SHARED
        src/test.cpp
)
ament_auto_add_executable(test_node
        src/test_node.cpp
)
target_link_libraries(test_node
        test_library
)

ament_auto_add_library(navigator_library SHARED
        src/plugins/action/compute_plan.cpp
)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_auto_package()