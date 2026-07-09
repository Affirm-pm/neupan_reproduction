find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(TEST_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}                  # test_helpers.hpp
    ${CMAKE_SOURCE_DIR}/include                  # finenav_core headers
    ${CMAKE_BINARY_DIR}/include                  # generated map_server_params.hpp
)

# --- test_observation_source ---
ament_add_gtest(test_observation_source map/test_observation_source.cpp)
target_include_directories(test_observation_source PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_observation_source ${PROJECT_NAME})
ament_target_dependencies(test_observation_source rclcpp rclcpp_lifecycle finenav_util)

# --- test_locked_map_view ---
ament_add_gtest(test_locked_map_view map/test_locked_map_view.cpp)
target_include_directories(test_locked_map_view PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_locked_map_view ${PROJECT_NAME})
ament_target_dependencies(test_locked_map_view rclcpp rclcpp_lifecycle finenav_util)

# --- test_map_server ---
ament_add_gtest(test_map_server map/test_map_server.cpp)
target_include_directories(test_map_server PRIVATE ${TEST_INCLUDE_DIRS})
target_link_libraries(test_map_server ${PROJECT_NAME})
ament_target_dependencies(test_map_server rclcpp rclcpp_lifecycle finenav_util)
