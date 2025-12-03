cmake_minimum_required(VERSION 3.22)
project(finenav_core)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_planner_node src/fn_local_planner_node.cpp)
target_link_libraries(simple_planner_node Eigen3::Eigen)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_auto_package() # TODO: 之后讨论下是否添加 USE_SCOPED_HEADER_INSTALL_DIR 的问题