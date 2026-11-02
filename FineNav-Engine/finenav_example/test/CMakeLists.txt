ament_add_gtest(test_dummy_planners test_dummy_planners.cpp)
target_include_directories(test_dummy_planners PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../create_custom_planner/include)
target_link_libraries(test_dummy_planners
    dummy_line_planner
    dummy_line_planner_params
    dummy_follow_planner
    dummy_follow_planner_params)
ament_target_dependencies(test_dummy_planners finenav_core)
