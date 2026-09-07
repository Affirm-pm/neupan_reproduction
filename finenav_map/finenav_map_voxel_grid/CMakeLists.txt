cmake_minimum_required(VERSION 3.22)
project(finenav_map_voxel_grid)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

ament_auto_find_build_dependencies()

# ── Generate parameter struct from YAML ──────────────────────────────
generate_parameter_library(
    temporal_voxel_map_params
    param/temporal_voxel_map.yaml
)

# ── Library ──────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
    src/temporal_voxel_map/temporal_voxel_map.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    temporal_voxel_map_params
    ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    finenav_core
    finenav_visualizer
    rclcpp
)

# ── Install ──────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME} temporal_voxel_map_params
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(finenav_core finenav_visualizer rclcpp Eigen3 PCL)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
