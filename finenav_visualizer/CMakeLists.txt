cmake_minimum_required(VERSION 3.22)
project(finenav_visualizer)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
endif()

ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME} SHARED
    src/color.cpp
    src/marker_viz.cpp
    src/point_cloud_viz.cpp
    src/finenav_visualizer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    rclcpp
    visualization_msgs
    std_msgs
    sensor_msgs
    pcl_conversions
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/finenav_visualizer/
    DESTINATION include/finenav_visualizer
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp visualization_msgs std_msgs sensor_msgs Eigen3 PCL)

if (BUILD_TESTING)
    ament_add_gtest(marker_viz_test
        test/marker_viz_test.cpp
    )

    target_link_libraries(marker_viz_test ${PROJECT_NAME})
    ament_target_dependencies(marker_viz_test rclcpp visualization_msgs)

    ament_add_gtest(viz_channel_test
        test/viz_channel_test.cpp
    )

    target_link_libraries(viz_channel_test ${PROJECT_NAME})
    ament_target_dependencies(viz_channel_test rclcpp std_msgs)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
