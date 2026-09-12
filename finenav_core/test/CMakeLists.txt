find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(CORE_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include                  # finenav_core headers
    ${CMAKE_BINARY_DIR}/include                  # generated map_server_params.hpp
)
set(MAP_HELPERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/map)        # map/test_helpers.hpp
set(ROBOT_STATE_HELPERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/robot_state)

# --- test_observation_source ---
ament_add_gtest(test_observation_source map/test_observation_source.cpp)
target_include_directories(test_observation_source PRIVATE ${MAP_HELPERS_DIR} ${CORE_INCLUDE_DIRS})
target_link_libraries(test_observation_source ${PROJECT_NAME})
ament_target_dependencies(test_observation_source rclcpp rclcpp_lifecycle finenav_util)

# --- test_locked_map_view ---
ament_add_gtest(test_locked_map_view map/test_locked_map_view.cpp)
target_include_directories(test_locked_map_view PRIVATE ${MAP_HELPERS_DIR} ${CORE_INCLUDE_DIRS})
target_link_libraries(test_locked_map_view ${PROJECT_NAME})
ament_target_dependencies(test_locked_map_view rclcpp rclcpp_lifecycle finenav_util)

# --- test_robot_state_provider ---
ament_add_gtest(test_robot_state_provider robot_state/test_robot_state_provider.cpp)
target_include_directories(test_robot_state_provider PRIVATE ${ROBOT_STATE_HELPERS_DIR} ${CORE_INCLUDE_DIRS})
target_link_libraries(test_robot_state_provider ${PROJECT_NAME})

# --- test_map_server ---
ament_add_gtest(test_map_server map/test_map_server.cpp)
target_include_directories(test_map_server PRIVATE ${MAP_HELPERS_DIR} ${CORE_INCLUDE_DIRS})
target_link_libraries(test_map_server ${PROJECT_NAME})
ament_target_dependencies(test_map_server rclcpp rclcpp_lifecycle finenav_util)

# --- test_task_bus_intra ---
ament_add_gtest(test_task_bus_intra task_bus/test_task_bus_intra.cpp)
target_include_directories(test_task_bus_intra PRIVATE ${CORE_INCLUDE_DIRS})
target_link_libraries(test_task_bus_intra ${PROJECT_NAME})
ament_target_dependencies(test_task_bus_intra rclcpp finenav_interfaces)

# --- test_branch_table (header-only topology reduction) ---
ament_add_gtest(test_branch_table plan/test_branch_table.cpp)
target_include_directories(test_branch_table PRIVATE ${CORE_INCLUDE_DIRS})
target_link_libraries(test_branch_table ${PROJECT_NAME})

# --- test_algo_register_details (logger / visualizer injection probes) ---
ament_add_gtest(test_algo_register_details common/test_algo_register_details.cpp)
target_include_directories(test_algo_register_details PRIVATE ${CORE_INCLUDE_DIRS})
target_link_libraries(test_algo_register_details ${PROJECT_NAME})
ament_target_dependencies(test_algo_register_details rclcpp)
