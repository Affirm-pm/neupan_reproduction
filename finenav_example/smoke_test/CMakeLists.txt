add_executable(map_server_smoke_test main.cpp)
ament_target_dependencies(map_server_smoke_test rclcpp finenav_core finenav_util)
target_link_libraries(map_server_smoke_test Eigen3::Eigen)
install(TARGETS map_server_smoke_test DESTINATION lib/${PROJECT_NAME})
