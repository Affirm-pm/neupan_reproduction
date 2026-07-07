add_executable(hello_finenav main.cpp)
target_link_libraries(hello_finenav dummy_map dummy_planner Eigen3::Eigen)
ament_target_dependencies(hello_finenav rclcpp finenav_core finenav_util finenav_engine geometry_msgs)
install(TARGETS hello_finenav DESTINATION lib/${PROJECT_NAME})
