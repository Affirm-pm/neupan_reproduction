find_package(generate_parameter_library REQUIRED)

generate_parameter_library(dummy_map_params dummy_map.yaml)

add_library(dummy_map SHARED src/dummy_map.cpp)
target_include_directories(dummy_map PUBLIC include)
ament_target_dependencies(dummy_map finenav_core)
target_link_libraries(dummy_map dummy_map_params)

install(TARGETS dummy_map ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
