cmake_minimum_required(VERSION 3.22)
project(finenav_collision_model)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(finenav_core REQUIRED)
find_package(finenav_map_grid_map REQUIRED)

ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME} SHARED
    src/obb_collision_model.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
    finenav_core
    finenav_map_grid_map
)

# if (BUILD_TESTING)
#     find_package(ament_cmake_gtest REQUIRED)
#     ament_find_gtest()
#     add_subdirectory(test)
# endif ()

ament_auto_package()
