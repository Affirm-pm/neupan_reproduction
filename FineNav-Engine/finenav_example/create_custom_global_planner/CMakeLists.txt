find_package(finenav_route_planner REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
generate_parameter_library(finenav_global_planner_params finenav_global_planner.yaml)
generate_parameter_library(global_grid_world_view_params global_grid_world_view.yaml)

add_library(finenav_global_planner SHARED src/global_planner.cpp)
target_include_directories(finenav_global_planner PUBLIC include
    ../create_custom_neupan_planner/include ${finenav_route_planner_INCLUDE_DIRS})
# Upstream exports parameter targets and its algorithm library through different
# ament mechanisms. Name both: ament_target_dependencies alone selects only targets.
target_link_libraries(finenav_global_planner finenav_global_planner_params ${finenav_route_planner_LIBRARIES})
ament_target_dependencies(finenav_global_planner finenav_core finenav_route_planner finenav_visualizer)

add_library(global_grid_world_view SHARED src/global_grid_world_view.cpp)
target_include_directories(global_grid_world_view PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(global_grid_world_view global_grid_world_view_params finenav_global_planner
    neupan_world_view dummy_map dummy_map_params ${OpenCV_LIBS})
ament_target_dependencies(global_grid_world_view finenav_core nav_msgs)
finenav_register_algo_plugin(finenav_global_planner)
finenav_register_algo_plugin(global_grid_world_view)
install(TARGETS finenav_global_planner global_grid_world_view
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(PROGRAMS scripts/maze_global_map.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME test_maze_global_map
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_maze_global_map.py")
    ament_add_gtest(test_global_planner test/test_global_planner.cpp
        ENV "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_BINARY_DIR}/create_custom_map:$ENV{LD_LIBRARY_PATH}")
    target_link_libraries(test_global_planner global_grid_world_view)
endif()
