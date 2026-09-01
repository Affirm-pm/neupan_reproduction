add_library(dummy_mapview SHARED src/dummy_mapview.cpp)
target_include_directories(dummy_mapview PUBLIC include)
target_link_libraries(dummy_mapview
    dummy_map
    dummy_map_params
    dummy_planner
    dummy_planner_params)
ament_target_dependencies(dummy_mapview finenav_core)

install(TARGETS dummy_mapview ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
