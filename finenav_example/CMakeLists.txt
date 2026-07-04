cmake_minimum_required(VERSION 3.22)
project(finenav_example)

set(CMAKE_CXX_STANDARD 20)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(finenav_core REQUIRED)
find_package(finenav_util REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(generate_parameter_library REQUIRED)

include_directories(include)

add_subdirectory(smoke_test)
add_subdirectory(control_layer_smoke)
# TODO: re-enable when finenav_engine/navigator is fixed
# add_subdirectory(interface_demo)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_dependencies(generate_parameter_library)
ament_package()
