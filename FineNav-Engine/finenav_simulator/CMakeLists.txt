cmake_minimum_required(VERSION 3.22)
project(finenav_simulator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(point_cloud_converter_node
        src/point_cloud_converter.cpp
)
target_include_directories(point_cloud_converter_node PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
ament_target_dependencies(point_cloud_converter_node SYSTEM
        ros_gz_sim
        ros_gz_bridge
        pcl_conversions
        tf2_geometry_msgs
        nav_msgs
        geometry_msgs
        tf2_ros
)

add_executable(simulator_helper_node #TODO: executable
        src/simulator_helper_node.cpp
        src/keyboard_teleop.cpp
        src/odom_path_publisher.cpp
        src/ground_truth_map_localizer.cpp
)
target_include_directories(simulator_helper_node PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
ament_target_dependencies(simulator_helper_node SYSTEM
        ros_gz_sim
        ros_gz_bridge
        pcl_conversions
        tf2_geometry_msgs
        nav_msgs
        geometry_msgs
        tf2_ros
)

install(TARGETS
        point_cloud_converter_node
        simulator_helper_node
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY resource
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY scripts
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(ros_gz_sim ros_gz_bridge pcl_conversions tf2_geometry_msgs nav_msgs geometry_msgs tf2_ros)

ament_package()
