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
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(finenav_interfaces REQUIRED)
find_package(finenav_util REQUIRED)
find_package(finenav_core REQUIRED)

include_directories(include)

set(dependencies
        rclcpp
        rclcpp_action
        ament_index_cpp
        behaviortree_cpp
        generate_parameter_library
        eigen3_cmake_module
        geometry_msgs
        finenav_core
        finenav_interfaces
        finenav_util
)

# Library
generate_parameter_library(
        navigator_params
        src/finenav_navigator.yaml
)

add_library(finenav_navigator SHARED
        src/naviagtor.cpp
        src/behavior_tree_engine.cpp
        src/pipeline_controller.cpp
        src/chassis_command/chassis_cmd_vel.cpp
        src/chassis_command/chassis_cmd_vel_stamped.cpp
)
target_include_directories(finenav_navigator PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(finenav_navigator
        ${dependencies}
)
target_link_libraries(finenav_navigator
        navigator_params
        Eigen3::Eigen
)

# Action Plugins
add_library(finenav_publish_navigation_goal_behavior SHARED
        src/plugins/action/publish_navigation_goal.cpp
)
ament_target_dependencies(finenav_publish_navigation_goal_behavior
        ${dependencies}
)
target_compile_definitions(finenav_publish_navigation_goal_behavior PRIVATE BT_PLUGIN_EXPORT)

add_library(finenav_start_control_pipeline_branch_behavior SHARED
        src/plugins/action/start_control_pipeline_branch.cpp
)
ament_target_dependencies(finenav_start_control_pipeline_branch_behavior
        ${dependencies}
)
target_compile_definitions(finenav_start_control_pipeline_branch_behavior PRIVATE BT_PLUGIN_EXPORT)

add_library(finenav_start_control_layer_behavior SHARED
        src/plugins/action/start_control_layer.cpp
)
ament_target_dependencies(finenav_start_control_layer_behavior
        ${dependencies}
)
target_compile_definitions(finenav_start_control_layer_behavior PRIVATE BT_PLUGIN_EXPORT)

add_library(finenav_stop_control_layer_behavior SHARED
        src/plugins/action/stop_control_layer.cpp
)
ament_target_dependencies(finenav_stop_control_layer_behavior
        ${dependencies}
)
target_compile_definitions(finenav_stop_control_layer_behavior PRIVATE BT_PLUGIN_EXPORT)

# Condition Plugins
add_library(finenav_is_goal_reached_condition SHARED
        src/plugins/condition/is_goal_reached.cpp
)
ament_target_dependencies(finenav_is_goal_reached_condition ${dependencies})
target_compile_definitions(finenav_is_goal_reached_condition PRIVATE BT_PLUGIN_EXPORT)

add_library(finenav_is_stuck_condition SHARED
        src/plugins/condition/is_stuck.cpp
)
ament_target_dependencies(finenav_is_stuck_condition ${dependencies})
target_compile_definitions(finenav_is_stuck_condition PRIVATE BT_PLUGIN_EXPORT)

add_library(finenav_is_near_point_condition SHARED
        src/plugins/condition/is_near_point.cpp
)
ament_target_dependencies(finenav_is_near_point_condition ${dependencies})
target_compile_definitions(finenav_is_near_point_condition PRIVATE BT_PLUGIN_EXPORT)


# StopChassisMotion Plugin
add_library(finenav_stop_chassis_motion_behavior SHARED
        src/plugins/action/stop_chassis_motion.cpp
)
ament_target_dependencies(finenav_stop_chassis_motion_behavior
        ${dependencies}
)
target_compile_definitions(finenav_stop_chassis_motion_behavior PRIVATE BT_PLUGIN_EXPORT)

# Control & Decorator Plugins (pure BT nodes, no ROS params needed)
add_library(finenav_recovery_node SHARED
        src/plugins/control/recovery_node.cpp
)
ament_target_dependencies(finenav_recovery_node
        ${dependencies}
)
target_compile_definitions(finenav_recovery_node PRIVATE BT_PLUGIN_EXPORT)

add_library(finenav_round_robin_node SHARED
        src/plugins/control/round_robin_node.cpp
)
ament_target_dependencies(finenav_round_robin_node
        ${dependencies}
)
target_compile_definitions(finenav_round_robin_node PRIVATE BT_PLUGIN_EXPORT)

add_library(finenav_rate_controller SHARED
        src/plugins/decorator/rate_controller.cpp
)
ament_target_dependencies(finenav_rate_controller
        ${dependencies}
)
target_compile_definitions(finenav_rate_controller PRIVATE BT_PLUGIN_EXPORT)

add_library(finenav_keep_for_duration SHARED
        src/plugins/decorator/keep_for_duration.cpp
)
ament_target_dependencies(finenav_keep_for_duration
        ${dependencies}
)
target_compile_definitions(finenav_keep_for_duration PRIVATE BT_PLUGIN_EXPORT)

# DynamicGoalFollower Plugin (ROS node params needed)
add_library(finenav_dynamic_goal_follower SHARED
        src/plugins/decorator/dynamic_goal_follower.cpp
)
ament_target_dependencies(finenav_dynamic_goal_follower
        ${dependencies}
)
target_compile_definitions(finenav_dynamic_goal_follower PRIVATE BT_PLUGIN_EXPORT)

# Install — main library
install(TARGETS finenav_navigator navigator_params
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install — BT plugins (separate subdirectory so the scanner doesn't pick up non-plugin .so)
install(TARGETS
        finenav_publish_navigation_goal_behavior
        finenav_start_control_pipeline_branch_behavior
        finenav_start_control_layer_behavior
        finenav_stop_control_layer_behavior
        finenav_stop_chassis_motion_behavior
        finenav_is_goal_reached_condition
        finenav_is_stuck_condition
        finenav_is_near_point_condition
        finenav_dynamic_goal_follower
        finenav_recovery_node
        finenav_round_robin_node
        finenav_rate_controller
        finenav_keep_for_duration
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

# Export
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_libraries(finenav_navigator)
ament_export_dependencies(${dependencies} Eigen3)

ament_package()
