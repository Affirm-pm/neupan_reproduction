add_library(dummy_world_view SHARED src/dummy_world_view.cpp)
target_include_directories(dummy_world_view PUBLIC include)
target_link_libraries(dummy_world_view
    dummy_map
    dummy_map_params
    dummy_planner
    dummy_planner_params)
ament_target_dependencies(dummy_world_view finenav_core)

install(TARGETS dummy_world_view ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
