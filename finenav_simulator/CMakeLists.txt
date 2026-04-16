cmake_minimum_required(VERSION 3.22)
project(finenav_simulator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_executable(point_cloud_converter_node
        src/point_cloud_converter.cpp
)

ament_auto_add_executable(simulator_helper_node #TODO: executable
        src/simulator_helper_node.cpp
        src/keyboard_teleop.cpp
        src/odom_path_publisher.cpp
        src/ground_truth_map_localizer.cpp
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
        launch
        resource
        config
        scripts
)
