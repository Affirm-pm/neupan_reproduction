cmake_minimum_required(VERSION 3.22)
project(finenav_map_occ_grid_2d)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# ── 从 YAML 生成参数结构体（occupancy_grid_2d::Params / ParamListener）──
generate_parameter_library(
    occupancy_grid_2d_params
    param/occupancy_grid_2d.yaml
)

# ── 地图库 ────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
    src/occupancy_grid_2d.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/occupancy_grid_2d_params_parameters>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    occupancy_grid_2d_params
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    finenav_core
    finenav_util
    nav_msgs
    rclcpp
)

# ── 安装 ─────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME} occupancy_grid_2d_params
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(finenav_core finenav_util nav_msgs rclcpp)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
