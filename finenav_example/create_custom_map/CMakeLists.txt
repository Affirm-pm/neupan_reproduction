generate_parameter_library(dummy_map_params dummy_map.yaml)

add_executable(create_custom_map create_custom_map.cpp)
target_include_directories(create_custom_map PRIVATE ${CMAKE_SOURCE_DIR}/include)
ament_target_dependencies(create_custom_map rclcpp finenav_core finenav_util finenav_engine)
target_link_libraries(create_custom_map dummy_map_params Eigen3::Eigen)
install(TARGETS create_custom_map DESTINATION lib/${PROJECT_NAME})
