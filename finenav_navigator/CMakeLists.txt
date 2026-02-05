cmake_minimum_required(VERSION 3.22)
project(finenav_navigator)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(finenav_msgs REQUIRED)
find_package(finenav_util REQUIRED)

include_directories(include)

set(dependencies
        rclcpp
        rclcpp_action
        ament_index_cpp
        behaviortree_cpp
        generate_parameter_library
        finenav_msgs
        finenav_util
)

# Library
generate_parameter_library(
        navigator_params
        src/finenav_navigator.yaml
)

add_library(finenav_navigator STATIC
        src/naviagtor.cpp
        src/behavior_tree_engine.cpp
)
ament_target_dependencies(finenav_navigator
        ${dependencies}
)
target_link_libraries(finenav_navigator
        navigator_params
)

# Executable
add_executable(test_node
        src/test_node.cpp
)
target_link_libraries(test_node
        finenav_navigator
)

# Plugins
add_library(finenav_compute_plan_behavior SHARED
        src/plugins/action/compute_plan.cpp
)
ament_target_dependencies(finenav_compute_plan_behavior
        ${dependencies}
)
target_compile_definitions(finenav_compute_plan_behavior PRIVATE BT_PLUGIN_EXPORT)

# Install
install(TARGETS test_node
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS finenav_compute_plan_behavior
        ARCHIVE DESTINATION lib/plugins
        LIBRARY DESTINATION lib/plugins
        RUNTIME DESTINATION bin
)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

# Export
ament_export_include_directories(include)
ament_export_libraries(
        finenav_navigator
        finenav_compute_plan_behavior
)
ament_export_dependencies(${dependencies})

ament_package()