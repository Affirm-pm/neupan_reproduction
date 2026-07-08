cmake_minimum_required(VERSION 3.22)
project(finenav_localizer)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(finenav_core REQUIRED)
find_package(finenav_util REQUIRED)
find_package(generate_parameter_library REQUIRED)

# C++ dependencies
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(include)

# Generate parameter library for FineNavLocalizer
generate_parameter_library(
        localizer_params
        src/finenav_localizer.yaml
)

# FineNav Localizer (ESKF-based localization module)
add_library(${PROJECT_NAME} SHARED
        src/finenav_localizer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        tf2_ros
        tf2_eigen
        geometry_msgs
        finenav_core
        finenav_util
        generate_parameter_library
)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        Sophus::Sophus
        localizer_params
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
    add_subdirectory(eval)
endif ()

install(TARGETS ${PROJECT_NAME} localizer_params
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
        rclcpp
        tf2_ros
        tf2_eigen
        geometry_msgs
        finenav_core
        finenav_util
        Eigen3
        Sophus
        generate_parameter_library
)
ament_package()
