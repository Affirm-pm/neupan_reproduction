add_executable(hello_finenav hello_finenav.cpp)

ament_target_dependencies(hello_finenav rclcpp finenav_engine_cpp)
install(TARGETS hello_finenav DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME}/getting_started)
