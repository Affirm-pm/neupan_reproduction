find_package(generate_parameter_library REQUIRED)

generate_parameter_library(dummy_planner_params dummy_planner.yaml)

add_library(dummy_planner SHARED src/dummy_planner.cpp)
target_include_directories(dummy_planner PUBLIC include)
ament_target_dependencies(dummy_planner finenav_core)
target_link_libraries(dummy_planner dummy_planner_params)

install(TARGETS dummy_planner ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
