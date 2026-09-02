cmake_minimum_required(VERSION 3.22)
project(finenav_motion_decorator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(generate_parameter_library REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

generate_parameter_library(seek_to_nearest_point_params seek_to_nearest_point.yaml)
generate_parameter_library(smooth_path_params smooth_path.yaml)
generate_parameter_library(trim_by_distance_params trim_by_distance.yaml)
generate_parameter_library(trim_by_duration_params trim_by_duration.yaml)
generate_parameter_library(trim_by_count_params trim_by_count.yaml)
generate_parameter_library(trim_by_aabb_params trim_by_aabb.yaml)

ament_auto_add_library(${PROJECT_NAME} SHARED
    src/trim_by_count.cpp
    src/trim_by_duration.cpp
    src/trim_by_distance.cpp
    src/seek_to_nearest_point.cpp
    src/trim_by_aabb.cpp
    src/smooth_path.cpp
)

target_link_libraries(${PROJECT_NAME}
    seek_to_nearest_point_params
    smooth_path_params
    trim_by_distance_params
    trim_by_duration_params
    trim_by_count_params
    trim_by_aabb_params
)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
