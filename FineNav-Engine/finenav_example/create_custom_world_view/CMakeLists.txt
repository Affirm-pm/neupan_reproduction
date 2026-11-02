generate_parameter_library(dummy_line_world_view_params dummy_line_world_view.yaml)
generate_parameter_library(dummy_follow_world_view_params dummy_follow_world_view.yaml)

add_library(dummy_world_view SHARED src/dummy_world_view.cpp)
target_include_directories(dummy_world_view PUBLIC include)
target_link_libraries(dummy_world_view
    dummy_line_world_view_params
    dummy_follow_world_view_params
    dummy_map
    dummy_map_params
    dummy_line_planner
    dummy_line_planner_params
    dummy_follow_planner
    dummy_follow_planner_params)
ament_target_dependencies(dummy_world_view finenav_core)

install(TARGETS dummy_world_view ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

finenav_register_algo_plugin(dummy_world_view)
