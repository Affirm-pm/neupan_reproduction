# Standalone evaluation executable — no ROS dependencies.
# Links directly against algorithm source files for isolated testing.
add_executable(route_planner_eval
    eval_scenario.cpp
    ${PROJECT_SOURCE_DIR}/src/astar/astar_algorithm.cpp
    ${PROJECT_SOURCE_DIR}/src/dstarlite/dstarlite_algorithm.cpp
)

target_include_directories(route_planner_eval PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../test   # grid_map.hpp
)

# Pass the eval directory path to C++ so plot.py can be found regardless
# of where the project is cloned.
target_compile_definitions(route_planner_eval PRIVATE
    EVAL_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
