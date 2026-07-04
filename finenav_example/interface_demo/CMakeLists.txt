find_package(finenav_engine REQUIRED)

generate_parameter_library(test_map_params test_map.yaml)
generate_parameter_library(test_planner_params test_planner.yaml)

add_executable(interface_demo test.cpp)
ament_target_dependencies(interface_demo rclcpp finenav_core finenav_util finenav_engine)
target_link_libraries(interface_demo test_map_params test_planner_params Eigen3::Eigen)
install(TARGETS interface_demo DESTINATION lib/${PROJECT_NAME})
