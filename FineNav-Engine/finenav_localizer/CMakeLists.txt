cmake_minimum_required(VERSION 3.22)
project(finenav_localizer)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(finenav_core REQUIRED)
find_package(finenav_util REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

generate_parameter_library(
        localizer_params
        src/finenav_localizer.yaml
)

add_library(${PROJECT_NAME} SHARED
        src/finenav_localizer.cpp
        src/sources/world_anchor.cpp
        src/sources/integrator.cpp
        src/sources/velocity_provider.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        tf2_ros
        geometry_msgs
        nav_msgs
        finenav_core
        finenav_util
        generate_parameter_library
)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
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
        geometry_msgs
        nav_msgs
        finenav_core
        finenav_util
        Eigen3
        generate_parameter_library
)
ament_package()
